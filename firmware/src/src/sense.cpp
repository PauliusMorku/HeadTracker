/*
 * This file is part of the Head Tracker distribution (https://github.com/dlktdr/headtracker)
 * Copyright (c) 2021 Cliff Blackburn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "sense.h"

#include <float.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "MadgwickAHRS/MadgwickAHRS.h"
#include "analog.h"
#include "ble.h"
#include "defines.h"
#include "filters/SF1eFilter.h"
#include "io.h"
#include "joystick.h"

#include "htmain.h"
#include "pmw.h"
#include "soc_flash.h"
#include "trackersettings.h"
#include "uart_mode.h"

#if defined(HAS_APDS9960)
#include "APDS9960/APDS9960.h"
#endif
#if defined(HAS_LSM9DS1)
#include "LSM9DS1/LSM9DS1.h"
#endif
#if defined(HAS_LSM6DS3)
#include "LSMCommon/lsm_common.h"
#endif
#if defined(HAS_MPU6500)
#include "MPU6xxx/inv_mpu.h"
#endif
#if defined(HAS_MPU6886)
#include "MPU6886/MPU6886.h"
#endif
#if defined(HAS_QMC5883)
#include "QMC5883/qmc5883.h"
#endif
#if defined(HAS_BMI270)
#include "bmi270.h"
#include "BMI270/bmi270_common.h"
#endif
#if defined(HAS_BMM150)
#include "BMM150/bmm150.h"
#include "BMM150/bmm150_common.h"
#endif

#define CRSF_ACTUAL_RATE_CHANNEL 12  // Channel for displaying actual CRSF transmission rate in OSD
#define DAMPENING_SELECTION_CHANNEL 7 // Dampening selection channel is also used for tilt when TRP disabled

#define PROXIMITY_UPDATE_INTERVAL 10  // Proximity sensor update interval in main thread cycles

// #define DEBUG_SENSOR_RATES

// Analog Filters
SF1eFilter *anFilter[AN_CH_CNT];

// Battery Voltage Filter
SF1eFilter *anVoltFilter;

// ============================================================================
// Data Structures
// ============================================================================

typedef union {
  struct {
    float x, y, z;
  };
  float data[3];
} axis_t;

typedef struct {
    axis_t acc;        // Calibrated accelerometer for auxiliary functions
    axis_t gyr;        // Calibrated gyroscope for auxiliary functions
    axis_t raw_acc;    // Raw accelerometer for double-tap and calibration
    axis_t raw_gyr;    // Raw gyroscope for calibration
    uint16_t tilt_val; // Final tilt channel value
    uint16_t roll_val; // Final roll channel value
    uint16_t pan_val;  // Final pan channel value
    uint16_t roll_ui;  // Roll value for reset-on-tilt detection
    uint32_t actual_rate; // Actual CRSF transmission rate
} shared_thread_data_t;  // Shared between TRP and main threads

// ============================================================================
// Shared State
// ============================================================================

// Thread-shared sensor data and TRP output (populated by TRP thread, read by main thread)
static shared_thread_data_t shared_thread_data;

// Recenter request flag - set by main thread, cleared by TRP thread
static volatile bool recenter_requested = false;

// Thread-shared output state
static volatile bool trpOutputEnabled = true;
static volatile bool gyroCalibrated = false;

// Input Channel Data (used across main thread functions)
static uint16_t ppm_in_chans[16];
static uint16_t uart_in_chans[16];
static uint16_t bt_chans[TrackerSettings::BT_CHANNELS];
static float bt_chansf[TrackerSettings::BT_CHANNELS];

// Output channel data
static uint16_t channel_data[16];

Madgwick madgwick;

static bool hasAcc = false;
static bool hasGyr = false;
static bool hasMag = false;

#if defined(HAS_APDS9960)
static bool blesenseboard = false;
static bool lastproximity = false;
#endif

#if defined(HAS_LSM6DS3)
stmdev_ctx_t dev_ctx;
#endif

#if defined(HAS_LSM9DS1)
LSM9DS1Class IMU;
#endif

#if defined(HAS_BMI270)
struct bmi2_dev bmi2_dev;
#endif

#if defined(HAS_BMM150)
struct bmm150_dev bmm1_dev;
#endif

#if defined(HAS_MPU6886)
MPU6886 mpu6886;
#endif

#define SENSOR_VALUE_TO_FLOAT(x) ((float)x.val1 + (float)x.val2 / 1000000.0f)

// Initial Orientation Data+Vars
#define MADGINIT_ACCEL 0x01
#define MADGINIT_MAG 0x02
#define MADGINIT_READY (MADGINIT_ACCEL | MADGINIT_MAG)

LOG_MODULE_REGISTER(sensors);

static int madgreads = 0;
static uint8_t madgsensbits = 0;
static volatile bool firstrun = true;
static axis_t aacc = {{0, 0, 0}};
static axis_t amag = {{0, 0, 0}};

static struct k_poll_signal trpThreadRunSignal = K_POLL_SIGNAL_INITIALIZER(trpThreadRunSignal);
struct k_poll_event trpRunEvents[1] = {
    K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &trpThreadRunSignal),
};

static struct k_poll_signal channelThreadRunSignal =
    K_POLL_SIGNAL_INITIALIZER(channelThreadRunSignal);
struct k_poll_event channelRunEvents[1] = {
    K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
                             &channelThreadRunSignal),
};

// ============================================================================
// Helper Functions
// ============================================================================

void gyroCalibrate();
void detectDoubleTap();
void readSensors(axis_t& tacc, axis_t& tgyr, axis_t& tmag, bool& accValid, bool& gyrValid, bool& magValid);

inline float magnitude(const axis_t& v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline uint16_t clamp_channel(float value, uint16_t min_val, uint16_t max_val) {
  return MAX(MIN((uint16_t)value, max_val), min_val);
}

static inline bool assign_channel(uint16_t* channels, int channel, uint16_t value) {
    if (channel > 0 && channel <= 16) {
        channels[channel - 1] = value;
        return true;
    }
    return false;
}

static inline void processAnalogChannel(uint16_t* channels, int ch, float rawAnalog, float gain, float offset, int filterIndex) {
  float filtered = SF1eFilterDo(anFilter[filterIndex], rawAnalog);
  filtered *= gain;
  filtered += offset;
  filtered += TrackerSettings::MIN_PWM;
  filtered = MAX(TrackerSettings::MIN_PWM, MIN(TrackerSettings::MAX_PWM, filtered));
  assign_channel(channels, ch, filtered);
}


int sense_Init()
{
  // I2C device pointer (local to init function)
  const struct device *i2c_dev = nullptr;
  // If an I2C bus is defined in the device tree, initialize it
#if DT_NODE_EXISTS(DT_ALIAS(i2csensor))
  i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2csensor));
  if (!i2c_dev) {
    LOG_ERR("Could not get device binding for I2C");
    return false;
  }
  uint32_t i2c_cfg = 0;
  if(!i2c_get_config(i2c_dev, &i2c_cfg)){
    LOG_INF("I2C Config: 0x%x", i2c_cfg);
  }

  LOG_INF("Waiting for I2C Sensor Bus To Be Ready");
  while(!device_is_ready(DEVICE_DT_GET(DT_ALIAS(i2csensor)))) {
    k_msleep(10);
  }
  k_msleep(20);

  i2c_recover_bus(i2c_dev);
  LOG_INF("I2C Ready");
#endif

  // If an External I2C bus is defined in the device tree, initialize it
#if DT_NODE_EXISTS(DT_ALIAS(i2csensorext))
  const struct device *i2cext_dev = DEVICE_DT_GET(DT_ALIAS(i2csensorext));
  if (!i2cext_dev) {
    LOG_ERR("Could not get device binding for I2C");
    return false;
  }
  uint32_t i2cext_cfg = 0;
  if(!i2c_get_config(i2cext_dev, &i2cext_cfg)){
    LOG_INF("I2C Config: 0x%x", i2cext_cfg);
  }

  LOG_INF("Waiting for I2C External Sensor Bus To Be Ready");
  while(!device_is_ready(DEVICE_DT_GET(DT_ALIAS(i2csensorext)))) {
    k_msleep(10);
  }
  k_msleep(20);

  i2c_recover_bus(i2cext_dev);
  LOG_INF("I2CEXT Ready");
#endif

  // If a ACC&GYR sensor is defined in the device tree but not found
  // will cause a hardfault. If a mag is not found, the system will
  // continue to run without it. TODO.. notify in GUI
  hasAcc = false;
  hasGyr = false;
  hasMag = false;

#if defined(HAS_LSM9DS1)
  if (!IMU.begin()) {
    LOG_ERR("Failed to initialize LSM9DS1 Sensor");
    return -1;
  }
  hasAcc = true;
  hasGyr = true;
  hasMag = true;
#endif

#if defined(HAS_BMI270)
  int8_t rslt;
  uint8_t sensor_list[2] = {BMI2_ACCEL, BMI2_GYRO};
  rslt = bmi2_interface_init(&bmi2_dev, BMI2_I2C_INTF);
  bmi2_error_codes_print_result(rslt);

  rslt = bmi270_init(&bmi2_dev);
  bmi2_error_codes_print_result(rslt);

  if (rslt == BMI2_OK) {
    rslt = set_accel_gyro_config(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK) {
      rslt = bmi2_sensor_enable(sensor_list, 2, &bmi2_dev);
      bmi2_error_codes_print_result(rslt);
    }
    hasAcc = true;
    hasGyr = true;
  } else {
    LOG_ERR("Failed to initialize BMI270");
    return -1;
    }
#endif

#if defined(HAS_BMM150)
  /* Status of api are returned to this variable */
  int8_t rbslt;
  struct bmm150_settings settings;

  rbslt = bmm150_interface_selection(&bmm1_dev);
  bmm150_error_codes_print_result("bmm150_interface_selection", rbslt);
  if (rbslt == BMM150_OK) {
    rbslt = bmm150_init(&bmm1_dev);
    bmm150_error_codes_print_result("bmm150_init", rbslt);

    if (rbslt == BMM150_OK) {
      settings.pwr_mode = BMM150_POWERMODE_NORMAL;
      rslt = bmm150_set_op_mode(&settings, &bmm1_dev);
      bmm150_error_codes_print_result("set_op_mode", rbslt);
      if (rslt == BMM150_OK) {
        settings.data_rate = BMM150_DATA_RATE_30HZ;
        settings.xy_rep = 15;  // Repetitions for X/Y-axis (from datasheet)
        settings.z_rep = 15;  // Repetitions for Z-axis
        rslt = bmm150_set_sensor_settings(BMM150_SEL_XY_REP | BMM150_SEL_Z_REP | BMM150_SEL_DATA_RATE, &settings, &bmm1_dev);
        bmm150_error_codes_print_result("bmm150_set_sensor_settings", rslt);
        if (rslt == BMM150_OK) {
          LOG_INF("BMM150 Magnetometer Initialized");
          hasMag = true;
        }
      }
    }
  } else {
    LOG_ERR("Unable to init BMM150 - Continuing with no magnetomer\n");
  }
#endif

#if defined(HAS_LSM6DS3)
  if (initailizeLSM6DS3(&dev_ctx)) {
    LOG_ERR("Unable to init LSM6DS3\n");
    return -1;
  } else {
    hasGyr = true;
  }
#endif

#if defined(HAS_MPU6500)
  mpu_select_device(0);
  mpu_init_structures();
  mpu_init(NULL);
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_gyro_fsr(2000);
  mpu_set_accel_fsr(2);
  mpu_set_sample_rate(300);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  hasAcc = true;
  hasGyr = true;
#endif

#if defined(HAS_MPU6886)
  LOG_INF("MPU6886 Initializing");
  mpu6886.Init(); // ToDo add error checking
  hasAcc = true;
  hasGyr = true;
#endif

#if defined(HAS_APDS9960)
  // Initialize Gesture Sensor
  if (APDS.begin()) {
    blesenseboard = true;
    LOG_INF("APDS9960 Proximity Sensor Found");
  } else {
    blesenseboard = false;
    LOG_ERR("APDS9960 Proximity Sensor Not Found");
  }
#endif

#if defined(HAS_QMC5883)
  if(qmc5883Init()) {
    hasMag = true;
    LOG_INF("QMC5883 Magnetometer Initialized");
  }
  else
    LOG_ERR("QMC5883 Magnetometer Not Found");
#endif

  // No Gyro, no need to calibrate
  if(hasGyr == false) {
    gyroCalibrated = true;
    clearLEDFlag(LED_GYROCAL);
  }

  for (int i = 0; i < TrackerSettings::BT_CHANNELS; i++) {
    bt_chansf[i] = 0;
  }

  // Create analog filters
  for (int i = 0; i < AN_CH_CNT; i++) {
    anFilter[i] = SF1eFilterCreate(AN_FILT_FREQ, AN_FILT_MINCO, AN_FILT_SLOPE, AN_FILT_DERCO);
    SF1eFilterInit(anFilter[i]);
  }
  // Create battery voltage filter
  anVoltFilter = SF1eFilterCreate(AN_FILT_FREQ, AN_FILT_MINCO, AN_FILT_SLOPE, AN_FILT_DERCO);
  SF1eFilterInit(anVoltFilter);

  // Start TRP calculation thread (sensor fusion + UART output critical path)
  k_poll_signal_raise(&trpThreadRunSignal, 1);
  LOG_INF("TRP Thread Signal Raised");

  // Start channel management thread (all other I/O and channel processing)
  k_poll_signal_raise(&channelThreadRunSignal, 1);
  LOG_INF("Channel Thread Signal Raised");

  return 0;
}

//----------------------------------------------------------------------
// Main Thread
// Handles all non-critical channel I/O: PPM, BT, PWM, analog inputs,
// auxiliary functions, joystick output, proximity sensor, and button handling
//----------------------------------------------------------------------

void main_Thread()
{
  LOG_INF("Main Thread Loaded");
  while (1) {
    // Do not execute below until after initialization has happened
    k_poll(channelRunEvents, 1, K_FOREVER);

    if (k_sem_count_get(&flashWriteSemaphore) == 1) {
      k_msleep(10);
      continue;
    }

    int64_t usduration = micros64();

    /* ************************************************************
     *       Build channel data
     *
     * Build Channel Data
     *   1) Read TRP data from TRP thread
     *   2) Handle reset triggers (proximity, button, tilt)
     *   3) Set PPMin channels
     *   4) Set SBUSin channels
     *   5) Set received BT channels
     *   6) Set auxiliary functions
     *   7) Set analog channels
     *   8) Configure UART output
     *   9) Merge TRP data from TRP thread
     *  10) Output to PPMout
     *  11) Output to Bluetooth
     *  12) Output PWM channels
     *  13) Output to USB Joystick
     */

    // ========================================
    // 1) Read TRP Data from TRP Thread
    // ========================================
    
    // Note: For CRSF fast mode, TRP is written directly by TRP thread to channel_data
    // This path is for non-time-critical outputs (PPM, BT, PWM, Joystick)
    
    // Read TRP values with scheduler lock
    axis_t acc, gyr;
    uint16_t tilt_val, roll_val, pan_val, roll_ui, actual_rate;
    k_sched_lock();
    tilt_val = shared_thread_data.tilt_val;
    roll_val = shared_thread_data.roll_val;
    pan_val = shared_thread_data.pan_val;
    roll_ui = shared_thread_data.roll_ui;
    actual_rate = shared_thread_data.actual_rate;
    gyr = shared_thread_data.gyr;
    acc = shared_thread_data.acc;
    k_sched_unlock();

    // ========================================
    // 2) Reset Triggers & Button Handling
    // ========================================

    // Initialize channel data array
    uint16_t local_channel_data[16] = {0};

#if defined(HAS_APDS9960)
    // Reset Center on Proximity detection
    static int proximitycount = 0;
    static int minproximity = 100;  // Keeps smallest proximity read.
    static int maxproximity = 0;    // Keeps largest proximity value read.
    if (blesenseboard && proximitycount++ >= PROXIMITY_UPDATE_INTERVAL) {
      proximitycount = 0;
      if (trkset.getRstOnWave()) {
        // Reset on Proximity
        int proximity = APDS.readProximity();
        if (proximity != 1) {
          // Store High and Low Values, Generate reset thresholds
          maxproximity = MAX(proximity, maxproximity);
          minproximity = MIN(proximity, minproximity);
          int lowthreshold = minproximity + APDS_HYSTERISIS;
          int highthreshold = maxproximity - APDS_HYSTERISIS;

          // Don't allow reset if high and low thresholds are too close
          if (highthreshold - lowthreshold > APDS_HYSTERISIS * 2) {
            if (proximity < lowthreshold && lastproximity == false) {
              pressButton();
              LOG_INF("Reset center from a close proximity");
              lastproximity = true;
            } else if (proximity > highthreshold) {
              // Clear flag on proximity clear
              lastproximity = false;
            }
          }
        }
      }
    }
#endif

    // Button press handling
    
    static float pulsetimer = 0;
    static bool sendingresetpulse = false;
    
    // Check for short press (recenter)
    if (wasButtonPressed()) {
        LOG_INF("Reset Center Short Pressed");

        // Signal TRP thread to capture current orientation
        recenter_requested = true;
        
        // Start reset pulse on alert channel
        sendingresetpulse = true;
        pulsetimer = 0.0f;
        
        // Send to remote if in BT mode
        if (BTGetMode() == BTPARARMT) {
            BTRmtSendButtonPress(false);
        }
    }
    
    // Check for long press (toggle TRP output)
    if (wasButtonLongPressed()) {
        LOG_INF("Reset Center Long Pressed");
        trpOutputEnabled = !trpOutputEnabled;
        
        // Send to remote if in BT mode
        if (BTGetMode() == BTPARARMT) {
            BTRmtSendButtonPress(true);
        }
    }

    // Output reset pulse on alert channel
    int alertch = trkset.getAlertCh();
    if (alertch > 0) {
        if (sendingresetpulse) {
            // High pulse indicates reset/calibration
            local_channel_data[alertch - 1] = gyroCalibrated ? TrackerSettings::MAX_PWM : TrackerSettings::DEF_MAX_PWM;
            
            // Update pulse timer and end pulse after duration
            pulsetimer += (float)CALCULATE_PERIOD / 1000000.0f;
            if (pulsetimer > TrackerSettings::RECENTER_PULSE_DURATION) {
                sendingresetpulse = false;
            }
        } else {
            // Normal state - low indicates gyro calibration status
            local_channel_data[alertch - 1] = gyroCalibrated ? TrackerSettings::DEF_MIN_PWM : TrackerSettings::MIN_PWM;
        }
    }

    // Handle TRP output enable/disable based on long press mode setting
    static bool lastbutmode = true;
    bool buttonpresmode = trkset.getButLngPs();
    if (!buttonpresmode) {
        // Long press mode disabled - TRP always enabled
        trpOutputEnabled = true;
    } else if (!lastbutmode && buttonpresmode) {
        // Long press mode just enabled - disable TRP until user toggles it
        trpOutputEnabled = false;
    }
    lastbutmode = buttonpresmode;

    // Reset on tilt detection
    
    static bool doresetontilt = false;
    if (trkset.getRstOnTlt()) {
      static bool tiltpeak = false;
      static float resettime = 0.0f;
      enum {
        HITNONE,
        HITMIN,
        HITMAX,
      };
      static int minmax = HITNONE;
      if (roll_ui == trkset.getRll_Max()) {
        if (tiltpeak == false && minmax == HITNONE) {
          tiltpeak = true;
          minmax = HITMAX;
        } else if (minmax == HITMIN) {
          minmax = HITNONE;
          tiltpeak = false;
          doresetontilt = true;
        }

      } else if (roll_ui == trkset.getRll_Min()) {
        if (tiltpeak == false && minmax == HITNONE) {
          tiltpeak = true;
          minmax = HITMIN;
        } else if (minmax == HITMAX) {
          minmax = HITNONE;
          tiltpeak = false;
          doresetontilt = true;
        }
      }

      // If hit a max/min wait an amount of time and reset it
      if (tiltpeak == true) {
        resettime += (float)CALCULATE_PERIOD / 1000000.0f;
        if (resettime > TrackerSettings::RESET_ON_TILT_TIME) {
          tiltpeak = false;
          minmax = HITNONE;
          resettime = 0;
        }
      }
    }

    // Do the actual reset after a delay
    static float timetoreset = 0;
    if (doresetontilt) {
      if (timetoreset > TrackerSettings::RESET_ON_TILT_AFTER) {
        doresetontilt = false;
        timetoreset = 0;
        pressButton();
      }
      timetoreset += (float)CALCULATE_PERIOD / 1000000.0f;
    }

    // ========================================
    // 3) Read PPM Input Channels
    // ========================================

    PpmIn_execute();
    for (int i = 0; i < 16; i++)
      ppm_in_chans[i] = 0;
    int ppm_in_chcnt = PpmIn_getChannels(ppm_in_chans);
    if (ppm_in_chcnt >= 4 && ppm_in_chcnt <= 16) {
      for (int i = 0; i < MIN(ppm_in_chcnt, 16); i++) {
        local_channel_data[i] = ppm_in_chans[i];
      }
    }

    // ========================================
    // 4) Read UART Input Channels (SBUS/CRSF)
    // ========================================
    
    bool isUartValid = UartGetChannels(uart_in_chans);
    static bool lostmsgsent = false;
    static bool recmsgsent = false;
    if (!isUartValid) {
      if (!lostmsgsent) {
        LOG_ERR("Uart(SBUS/CRSF) Data Lost");
        lostmsgsent = true;
      }
      recmsgsent = false;
    } else {
      for (int i = 0; i < 16; i++) {
        local_channel_data[i] = uart_in_chans[i];
      }
      if (!recmsgsent) {
        LOG_DBG("Uart(SBUS/CRSF) Data Received");
        recmsgsent = true;
      }
      lostmsgsent = false;
    }

    // ========================================
    // 5) Read Bluetooth Input Channels
    // ========================================
    
    for (int i = 0; i < TrackerSettings::BT_CHANNELS; i++)
      bt_chans[i] = 0;
    for (int i = 0; i < TrackerSettings::BT_CHANNELS; i++) {
      uint16_t btvalue = BTGetChannel(i);
      if (btvalue > 0) {
        bt_chans[i] = btvalue;
        local_channel_data[i] = btvalue;
      }
    }

    // ========================================
    // 6) Set Auxiliary Function Channels
    // ========================================
    
    int aux0ch = trkset.getAux0Ch();
    int aux1ch = trkset.getAux1Ch();
    int aux2ch = trkset.getAux2Ch();
    float auxdata[8];
    if (aux0ch > 0 || aux1ch > 0 || aux2ch > 0) {
      float pwmrange = (TrackerSettings::MAX_PWM - TrackerSettings::MIN_PWM);
      auxdata[TrackerSettings::AUX_GYRX] = (gyr.x / 1000) * pwmrange + TrackerSettings::PPM_CENTER;
      auxdata[TrackerSettings::AUX_GYRY] = (gyr.y / 1000) * pwmrange + TrackerSettings::PPM_CENTER;
      auxdata[TrackerSettings::AUX_GYRZ] = (gyr.z / 1000) * pwmrange + TrackerSettings::PPM_CENTER;
      auxdata[TrackerSettings::AUX_ACCELX] = (acc.x / 2.0f) * pwmrange + TrackerSettings::PPM_CENTER;
      auxdata[TrackerSettings::AUX_ACCELY] = (acc.y / 2.0f) * pwmrange + TrackerSettings::PPM_CENTER;
      auxdata[TrackerSettings::AUX_ACCELZ] = (acc.z / 1.0f) * pwmrange + TrackerSettings::PPM_CENTER;
      auxdata[TrackerSettings::AUX_ACCELZO] = ((acc.z - 1.0f) / 2.0f) * pwmrange + TrackerSettings::PPM_CENTER;
      auxdata[TrackerSettings::BT_RSSI] = static_cast<float>(BTGetRSSI()) / 127.0f * pwmrange + TrackerSettings::MIN_PWM;

      assign_channel(local_channel_data, aux0ch, auxdata[trkset.getAux0Func()]);
      assign_channel(local_channel_data, aux1ch, auxdata[trkset.getAux1Func()]);
      assign_channel(local_channel_data, aux2ch, auxdata[trkset.getAux2Func()]);
    }

    // ========================================
    // 7) Read Analog Input Channels
    // ========================================
    
#ifdef ANVOLTMON
    float anbatt = SF1eFilterDo(anVoltFilter, analogRead(ANVOLTMON));
    anbatt *= ANVOLTMON_SCALE;
    anbatt += ANVOLTMON_OFFSET;
#endif

#ifdef AN0
    if (trkset.getAn0Ch() > 0) {
      processAnalogChannel(local_channel_data, trkset.getAn0Ch(), analogRead(AN0), trkset.getAn0Gain(), trkset.getAn0Off(), 0);
    }
#endif
#ifdef AN1
    if (trkset.getAn1Ch() > 0) {
      processAnalogChannel(local_channel_data, trkset.getAn1Ch(), analogRead(AN1), trkset.getAn1Gain(), trkset.getAn1Off(), 1);
    }
#endif
#ifdef AN2
    if (trkset.getAn2Ch() > 0) {
      processAnalogChannel(local_channel_data, trkset.getAn2Ch(), analogRead(AN2), trkset.getAn2Gain(), trkset.getAn2Off(), 2);
    }
#endif
#ifdef AN3
    if (trkset.getAn3Ch() > 0) {
      processAnalogChannel(local_channel_data, trkset.getAn3Ch(), analogRead(AN3), trkset.getAn3Gain(), trkset.getAn3Off(), 3);
    }
#endif

    // Apply dual-purpose dial logic
    static uint16_t last_dampening_value = TrackerSettings::PPM_CENTER;
    static bool last_trp_state = true;

    // Detect transition from enabled to disabled - capture dampening value
    if (last_trp_state && !trpOutputEnabled) {
      last_dampening_value = local_channel_data[DAMPENING_SELECTION_CHANNEL - 1];
    }
    last_trp_state = trpOutputEnabled;

    if (!trpOutputEnabled) {
      // TRP disabled: dial directly controls actual tilt channel
      uint16_t dial_value = local_channel_data[DAMPENING_SELECTION_CHANNEL - 1];
      int tilt_ch = trkset.getTltCh();
      
      if (tilt_ch > 0 && tilt_ch <= 16) {
        local_channel_data[tilt_ch - 1] = dial_value;
      }
      
      // Freeze dampening channel at last value
      assign_channel(local_channel_data, DAMPENING_SELECTION_CHANNEL, last_dampening_value);
    }

    // ========================================
    // 8) Configure UART Output
    // ========================================
    
    // If uart output set to CRSF_OUT, force channel 5 (AUX1/ARM) to high
    if (trkset.getUartMode() == TrackerSettings::UART_MODE_CRSFOUT) {
      if (trkset.getCh5Arm()) {
          local_channel_data[4] = 2000;
      }
    }

    // ========================================
    // 9) Merge TRP Data from TRP Thread
    // ========================================
    
    // Get channel assignments
    int tilt_ch = trkset.getTltCh();
    int roll_ch = trkset.getRllCh();
    int pan_ch = trkset.getPanCh();
    
    // Assign to local channel data
    if (trpOutputEnabled) {
      assign_channel(local_channel_data, tilt_ch, tilt_val);
    }
    assign_channel(local_channel_data, roll_ch, roll_val);
    assign_channel(local_channel_data, pan_ch, pan_val);
    assign_channel(local_channel_data, CRSF_ACTUAL_RATE_CHANNEL, actual_rate);

    // For non-CRSF modes: Copy non-TRP channels to channel_data (centering zeros)
    // For CRSF mode: TRP channels already written by TRP thread, we just handle other channels
    // Use k_sched_lock to prevent race condition with TRP thread
    int tlti = trkset.getTltCh() - 1;
    int rlli = trkset.getRllCh() - 1;
    int pani = trkset.getPanCh() - 1;

    k_sched_lock();
    for (int i = 0; i < 16; i++) {
        // Determine which channels to skip
        // Only skip tilt if TRP enabled - when disabled, main thread controls it via dial
        bool skip_tilt = (i == tlti) && trpOutputEnabled;
        bool skip_roll = (i == rlli);
        bool skip_pan = (i == pani);
        bool skip_rate = (i == CRSF_ACTUAL_RATE_CHANNEL - 1);
        
        if (skip_tilt || skip_roll || skip_pan || skip_rate) {
            continue;
        }
        
        // For other channels, center zeros before writing to channel_data
        channel_data[i] = (local_channel_data[i] == 0) ? TrackerSettings::PPM_CENTER : local_channel_data[i];
    }
    k_sched_unlock();

    // ========================================
    // 10) Output to PPM
    // ========================================
    
    PpmOut_execute();
    for (int i = 0; i < PpmOut_getChnCount(); i++) {
      uint16_t ppmout = local_channel_data[i];
      if (ppmout == 0) ppmout = TrackerSettings::PPM_CENTER;
      PpmOut_setChannel(i, ppmout);
    }

    // ========================================
    // 11) Output to Bluetooth
    // ========================================
    
    bool bleconnected = BTGetConnected();
    trkset.setDataBtAddr(BTGetAddress());
    for (int i = 0; i < TrackerSettings::BT_CHANNELS; i++) {
      BTSetChannel(i, local_channel_data[i]);
    }

    // ========================================
    // 12) Output to PWM Channels
    // ========================================
    
    int8_t pwmchs[4] = {trkset.getPwm0(), trkset.getPwm1(), trkset.getPwm2(), trkset.getPwm3()};
    for (int i = 0; i < 4; i++) {
      int pwmch = pwmchs[i] - 1;
      if (pwmch >= 0 && pwmch < 16) {
        uint16_t pwmout = local_channel_data[pwmch];
        if (pwmout == 0) pwmout = TrackerSettings::PPM_CENTER;
        setPWMValue(i, pwmout);
      }
    }

    // ========================================
    // 13) Output to USB Joystick
    // ========================================
    
    // Only 8 channels, half rate or USB is overwhelmed
    static uint32_t joystick_update = 0;
    if(joystick_update++ > 1) {
      joystick_update = 0;
      set_JoystickChannels(local_channel_data);
    }

    // ========================================
    // Update GUI Data
    // ========================================
    
    // Update GUI data (only if we can get the lock)
    if (k_mutex_lock(&data_mutex, K_NO_WAIT) == 0) {
      // PPM Input Values
      trkset.setDataPpmCh(ppm_in_chans);
      trkset.setDataBtCh(bt_chans);
      trkset.setDataUartCh(uart_in_chans);
      trkset.setDataChOut(local_channel_data);

      trkset.setDataTrpEnabled(trpOutputEnabled);
      trkset.setDataGyroCal(gyroCalibrated);

      // Bluetooth connected
      trkset.setDataBtCon(bleconnected);
      k_mutex_unlock(&data_mutex);
    }

    // ========================================
    // Thread Timing Control
    // ========================================
    
    // Adjust sleep for a more accurate period
    usduration = micros64() - usduration;
    if (CALCULATE_PERIOD - usduration <
        CALCULATE_PERIOD * 0.7) {  // Took a long time. Will crash if sleep is too short
      LOG_ERR("Main Thread Overrun %lld", usduration);
      k_usleep(CALCULATE_PERIOD);
    } else {
      k_usleep(CALCULATE_PERIOD - usduration);
    }

#if defined(DEBUG_SENSOR_RATES)
    static int mcount = 0;
    static int64_t mmic = millis64() + 1000;
    if (mmic < millis64()) {  // Every Second
      mmic = millis64() + 1000;
      LOG_INF("Main Thread Rate = %d", mcount);
      mcount = 0;
    }
    mcount++;
#endif
  }
}

//----------------------------------------------------------------------
// TRP Calculation Thread (Critical Path)
// Reads IMU sensors, performs sensor fusion, calculates TRP outputs,
// and writes directly to UART for minimum latency
//----------------------------------------------------------------------

void trp_Thread()
{
  LOG_INF("TRP Thread Loaded");
  
  int64_t senseUsDuration = 0;
  
  // Thread-local state
  axis_t racc = {{0, 0, 0}};
  axis_t rmag = {{0, 0, 0}};
  axis_t rgyr = {{0, 0, 0}};
  axis_t acc = {{0, 0, 0}};
  axis_t mag = {{0, 0, 0}};
  axis_t gyr = {{0, 0, 0}};
  
  while (1) {
    // Do not execute below until after initialization has happened
    k_poll(trpRunEvents, 1, K_FOREVER);

    if (k_sem_count_get(&flashWriteSemaphore) == 1) {
      k_msleep(10);
      continue;
    }

    senseUsDuration = micros64();

    // Setup Rotations
    float rotation[3] = {trkset.getRotX(), trkset.getRotY(), trkset.getRotZ()};

    // Read the data from the sensors
    axis_t tacc = {{0.0f, 0.0f, 0.0f}}, tgyr = {{0.0f, 0.0f, 0.0f}}, tmag = {{0.0f, 0.0f, 0.0f}};
    bool accValid = false;
    bool gyrValid = false;
    bool magValid = false;

    readSensors(tacc, tgyr, tmag, accValid, gyrValid, magValid);

    // -- Accelerometer
    if (accValid) {
      racc = tacc;

      acc.x = racc.x - trkset.getAccXOff();
      acc.y = racc.y - trkset.getAccYOff();
      acc.z = racc.z - trkset.getAccZOff();

      // Apply Rotation
      rotate(acc.data, rotation);

      // For intial orientation setup
      madgsensbits |= MADGINIT_ACCEL;
    }

    // --- Gyrometer Calcs
    if (gyrValid) {
      rgyr = tgyr;

      gyr.x = rgyr.x - trkset.getGyrXOff();
      gyr.y = rgyr.y - trkset.getGyrYOff();
      gyr.z = rgyr.z - trkset.getGyrZOff();

      // Apply Rotation
      rotate(gyr.data, rotation);
    }

    if (!trkset.getDisMag()) {
      if (magValid) {
        // --- Magnetometer Calcs
        rmag = tmag;

        // Get Soft Iron Offsets
        float magsioff[9];
        trkset.getMagSiOff(magsioff);

        // Calibrate Hard Iron Offsets - reuse tmag
        tmag.x -= trkset.getMagXOff();
        tmag.y -= trkset.getMagYOff();
        tmag.z -= trkset.getMagZOff();

        // Soft iron correction
        mag.x = (tmag.x * magsioff[0]) + (tmag.y * magsioff[1]) + (tmag.z * magsioff[2]);
        mag.y = (tmag.x * magsioff[3]) + (tmag.y * magsioff[4]) + (tmag.z * magsioff[5]);
        mag.z = (tmag.x * magsioff[6]) + (tmag.y * magsioff[7]) + (tmag.z * magsioff[8]);

        // Apply Rotation
        rotate(mag.data, rotation);

        // For initial orientation setup
        madgsensbits |= MADGINIT_MAG;
      }
    } else {
      mag = (axis_t){{0, 0, 0}};
      madgsensbits |= MADGINIT_MAG;
    }

    // Update shared sensor data for channel thread (no lock needed - TRP has higher priority)
    shared_thread_data.acc = acc;
    shared_thread_data.gyr = gyr;
    shared_thread_data.raw_acc = racc;
    shared_thread_data.raw_gyr = rgyr;

    // Run Gyro Calibration, only on good gyro data
    if (gyrValid) {
      gyroCalibrate();
    }

    // Only do this update after the first mag and accel data have been read.
    if (madgreads == 0) {
      if (madgsensbits == MADGINIT_READY) {
        madgsensbits = 0;
        madgreads++;
        aacc = acc;
        amag = mag;
      }

      // Average samples
    } else if (madgreads < MADGSTART_SAMPLES - 1) {
      if (madgsensbits == MADGINIT_READY) {
        madgsensbits = 0;
        madgreads++;
        float inv_count = 1.0f / madgreads;
        aacc.x += (acc.x - aacc.x) * inv_count;
        aacc.y += (acc.y - aacc.y) * inv_count;
        aacc.z += (acc.z - aacc.z) * inv_count;
        amag.x += (mag.x - amag.x) * inv_count;
        amag.y += (mag.y - amag.y) * inv_count;
        amag.z += (mag.z - amag.z) * inv_count;
      }

      // Got the averaged values, apply the initial orientation.
    } else if (madgreads == MADGSTART_SAMPLES - 1) {
      LOG_INF("Initial Orientation Set");
      // Pass averaged values
      madgwick.begin(aacc.x, aacc.y, aacc.z, amag.x, amag.y, amag.z);
      madgreads = MADGSTART_SAMPLES;
    }

    // Do the AHRS calculations
    float tilt = 0, roll = 0, pan = 0;
    uint16_t tiltout_ui = 0, rollout_ui = 0, panout_ui = 0;
    static float rolloffset = 0, panoffset = 0, tiltoffset = 0;
    if (madgreads == MADGSTART_SAMPLES) {
      // Period Between Samples
      float delttime = madgwick.deltatUpdate();

      madgwick.update(gyr.x * DEG_TO_RAD, gyr.y * DEG_TO_RAD, gyr.z * DEG_TO_RAD,
                      acc.x, acc.y, acc.z, mag.x, mag.y, mag.z, delttime);
      roll = madgwick.getPitch();
      tilt = madgwick.getRoll();
      pan = madgwick.getYaw();

      if (firstrun && pan != 0) {
        panoffset = pan;
        firstrun = false;
      }
    }

    // Check for recenter request from channel thread
    if (recenter_requested) {
        recenter_requested = false;
        LOG_DBG("Recentering to current orientation");
        rolloffset = roll;
        panoffset = pan;
        tiltoffset = tilt;
    }

    // Calculate outputs
    float tiltout = (tilt - tiltoffset) * trkset.getTlt_Gain() * (trkset.isTiltReversed() ? -1.0f : 1.0f);
    float rollout = (roll - rolloffset) * trkset.getRll_Gain() * (trkset.isRollReversed() ? -1.0f : 1.0f);
    float panout = normalize((pan - panoffset), -180, 180) * trkset.getPan_Gain() * (trkset.isPanReversed() ? -1.0f : 1.0f);

    // Convert to channel values
    tiltout_ui = clamp_channel(tiltout + trkset.getTlt_Cnt(), trkset.getTlt_Min(), trkset.getTlt_Max());
    rollout_ui = clamp_channel(rollout + trkset.getRll_Cnt(), trkset.getRll_Min(), trkset.getRll_Max());
    panout_ui = clamp_channel(panout + trkset.getPan_Cnt(), trkset.getPan_Min(), trkset.getPan_Max());

    // Apply TRP enable/disable
    int8_t tltch = trkset.getTltCh();
    int8_t rllch = trkset.getRllCh();
    int8_t panch = trkset.getPanCh();
    
    // When TRP disabled, main thread controls tilt channel via dial
    uint16_t final_tilt = trpOutputEnabled ? tiltout_ui : channel_data[tltch - 1];
    uint16_t final_roll = trpOutputEnabled ? rollout_ui : trkset.getRll_Cnt();
    uint16_t final_pan = trpOutputEnabled ? panout_ui : trkset.getPan_Cnt();

    static uint32_t crsfActualRate = 0;
    uint16_t rate_channel_val = crsfActualRate / 2 + 1500;

    // CRITICAL FAST PATH: Write TRP directly to shared channel_data for minimum latency
    // This bypasses message queues and main thread for time-critical UART output
    if (tltch > 0 && tltch <= 16) channel_data[tltch - 1] = final_tilt;
    if (rllch > 0 && rllch <= 16) channel_data[rllch - 1] = final_roll;
    if (panch > 0 && panch <= 16) channel_data[panch - 1] = final_pan;
    channel_data[CRSF_ACTUAL_RATE_CHANNEL - 1] = rate_channel_val;

    // Update shared TRP output for main thread (no lock needed - TRP has higher priority)
    shared_thread_data.tilt_val = final_tilt;
    shared_thread_data.roll_val = final_roll;
    shared_thread_data.pan_val = final_pan;
    shared_thread_data.roll_ui = rollout_ui;
    shared_thread_data.actual_rate = rate_channel_val;

    // CRITICAL FAST PATH: CRSF output directly from TRP thread for minimum latency
    // No message queues, no waiting for channel thread
    uint32_t sensorPeriod = SENSOR_PERIOD;
    if (trkset.getUartMode() == TrackerSettings::UART_MODE_CRSFOUT) {
      uint32_t crsfRate = ((trkset.getCrsfTxRate()+1) * 2);
      sensorPeriod = (1.0f / (float)crsfRate) * 1.0e6f;

      // FAST PATH: Use channel_data directly (already has TRP values written above)
      UartSetChannels(channel_data);
    }

    // Update GUI data (only if we can get the lock)
    if (k_mutex_lock(&data_mutex, K_NO_WAIT) == 0) {
      // Raw values for calibration
      trkset.setDataAccX(racc.x);
      trkset.setDataAccY(racc.y);
      trkset.setDataAccZ(racc.z);

      trkset.setDataGyroX(rgyr.x);
      trkset.setDataGyroY(rgyr.y);
      trkset.setDataGyroZ(rgyr.z);

      trkset.setDataMagX(rmag.x);
      trkset.setDataMagY(rmag.y);
      trkset.setDataMagZ(rmag.z);

      trkset.setDataOff_AccX(acc.x);
      trkset.setDataOff_AccY(acc.y);
      trkset.setDataOff_AccZ(acc.z);

      trkset.setDataOff_GyroX(gyr.x);
      trkset.setDataOff_GyroY(gyr.y);
      trkset.setDataOff_GyroZ(gyr.z);

      trkset.setDataOff_MagX(mag.x);
      trkset.setDataOff_MagY(mag.y);
      trkset.setDataOff_MagZ(mag.z);

      // Orientation data
      trkset.setDataTilt(tilt);
      trkset.setDataRoll(roll);
      trkset.setDataPan(pan);

      trkset.setDataTiltOff(tilt - tiltoffset);
      trkset.setDataRollOff(roll - rolloffset);
      trkset.setDataPanOff(normalize(pan - panoffset, -180, 180));

      trkset.setDataTiltOut(tiltout_ui);
      trkset.setDataRollOut(rollout_ui);
      trkset.setDataPanOut(panout_ui);

      float *qd = madgwick.getQuat();
      trkset.setDataQuat(qd);

      k_mutex_unlock(&data_mutex);
    }

    // Adjust sleep for a more accurate period
    senseUsDuration = micros64() - senseUsDuration;
    if (sensorPeriod - senseUsDuration <
        sensorPeriod * 0.4) {  // Took a long time. Will crash if sleep is too short
      LOG_ERR("TRP Thread Overrun %lld", senseUsDuration);
      k_usleep(sensorPeriod);
    } else {
      k_usleep(sensorPeriod - senseUsDuration);
    }

    static int mcount = 0;
    static int64_t mmic = millis64() + 1000;
    if (mmic < millis64()) {  // Every Second
      mmic = millis64() + 1000;
      crsfActualRate = mcount;
      mcount = 0;
    }
    if (accValid) mcount++;
  }  // END THREAD
}

void detectDoubleTap()
{
  // Get sensor data from shared state
  axis_t local_racc;
  k_sched_lock();
  local_racc = shared_thread_data.raw_acc;
  k_sched_unlock();
  
  static float last_acc_mag = 0;
  static uint64_t lasttaptime = 0;
  static uint64_t lasttime = 0;
  uint64_t time = millis64();
  uint64_t timediff;

  float deltatime = (float)(time - lasttime) / 1000.0f;
  if (deltatime == 0.0f) return;
  lasttime = time;

  float acc_magnitude = magnitude(local_racc);
  float acc_dif = (acc_magnitude - last_acc_mag) / deltatime;
  last_acc_mag = acc_magnitude;

  if (acc_dif > trkset.getRstOnDblTapThres())
  {
      timediff = time - lasttaptime;
      LOG_DBG("Tap detected, mag=%.1f, time=%lld, timediff=%lld", (double)acc_dif, time, timediff);

      if (timediff > trkset.getRstOnDblTapMin() && timediff < trkset.getRstOnDblTapMin() + trkset.getRstOnDblTapMax())
      {
        LOG_INF("Double Tap detected !!!");
        pressButton();
      }

      lasttaptime = time;
  }
}

void gyroCalibrate()
{
  // Get sensor data from shared state (no lock needed - TRP has higher priority)
  axis_t local_rgyr = shared_thread_data.raw_gyr;
  axis_t local_racc = shared_thread_data.raw_acc;
  
  static float last_gyro_mag = 0;
  static float last_acc_mag = 0;
  static axis_t filt_gyro = {{0, 0, 0}};
  static bool sent_gyro_cal_msg = false;
  static uint32_t filter_samples = 0;
  static uint64_t lasttime = 0;

  if(gyroCalibrated)
    return;

  uint64_t time = micros64();
  if (lasttime == 0) {  // Skip first run
    lasttime = time;
    return;
  }
  float deltatime = (float)(time - lasttime) / 1000000.0f;
  if (deltatime == 0.0f) return;
  lasttime = time;

  float gyro_magnitude = magnitude(local_rgyr);
  float acc_magnitude = magnitude(local_racc);
  float gyro_dif = (gyro_magnitude - last_gyro_mag) / deltatime;
  last_gyro_mag = gyro_magnitude;
  float acc_dif = (acc_magnitude - last_acc_mag) / deltatime;
  last_acc_mag = acc_magnitude;

  // Is Gyro and Accelerometer stable?
  if (fabsf(gyro_dif) < GYRO_STABLE_DIFF && fabsf(acc_dif) < ACC_STABLE_DIFF) {
    // First run, preload filter
    if (filter_samples == 0) {
      filt_gyro.x = local_rgyr.x;
      filt_gyro.y = local_rgyr.y;
      filt_gyro.z = local_rgyr.z;
      sent_gyro_cal_msg = false;
      filter_samples++;
    } else if (filter_samples < GYRO_STABLE_SAMPLES) {
      static const float GYRO_FILTER_COEFF_OLD = 1.0f - GYRO_SAMPLE_WEIGHT;
      static const float GYRO_FILTER_COEFF_NEW = GYRO_SAMPLE_WEIGHT;
      
      filt_gyro.x = (GYRO_FILTER_COEFF_OLD * filt_gyro.x) + (GYRO_FILTER_COEFF_NEW * local_rgyr.x);
      filt_gyro.y = (GYRO_FILTER_COEFF_OLD * filt_gyro.y) + (GYRO_FILTER_COEFF_NEW * local_rgyr.y);
      filt_gyro.z = (GYRO_FILTER_COEFF_OLD * filt_gyro.z) + (GYRO_FILTER_COEFF_NEW * local_rgyr.z);
      filter_samples++;
    } else if (filter_samples == GYRO_STABLE_SAMPLES) {
      LOG_INF("Gyro Calibrated, x=%.3f,y=%.3f,z=%.3f", (double)filt_gyro.x, (double)filt_gyro.y,
              (double)filt_gyro.z);
      gyroCalibrated = true;
      clearLEDFlag(LED_GYROCAL);

      k_mutex_lock(&data_mutex, K_FOREVER);
      axis_t current_off = {{0, 0, 0}};
      // Get the current Gyro Offset Values
      current_off.x = trkset.getGyrXOff();
      current_off.y = trkset.getGyrYOff();
      current_off.z = trkset.getGyrZOff();

      // Set the new Gyro Offset Values
      trkset.setGyrXOff(filt_gyro.x);
      trkset.setGyrYOff(filt_gyro.y);
      trkset.setGyrZOff(filt_gyro.z);
      k_mutex_unlock(&data_mutex);

      if (fabsf(current_off.x - filt_gyro.x) > GYRO_FLASH_IF_OFFSET ||
          fabsf(current_off.y - filt_gyro.y) > GYRO_FLASH_IF_OFFSET ||
          fabsf(current_off.z - filt_gyro.z) > GYRO_FLASH_IF_OFFSET) {
        if (!sent_gyro_cal_msg) {
          k_sem_give(&saveToFlash_sem);
          LOG_INF("Gyro calibration differs from saved value. Updating flash, x=%.3f,y=%.3f,z=%.3f",
               (double)filt_gyro.x, (double)filt_gyro.y, (double)filt_gyro.z);
          sent_gyro_cal_msg = true;
        }
      }
      filter_samples++;
    }
  } else {
    filter_samples = 0;
  }

  // Output in CSV format for determining limits
  // printk("%.4f,%.2f,%.2f\n", (float)time / 1000000.0f, gyro_dif, acc_dif);
}

// FROM https://stackoverflow.com/questions/1628386/normalise-orientation-between-0-and-360
// Normalizes any number to an arbitrary range
// by assuming the range wraps around when going below min or above max
float normalize(const float value, const float start, const float end)
{
  const float width = end - start;
  if (width == 0.0f) return start;  // Avoid division by zero
  const float offsetValue = value - start;  // value relative to 0

  return (offsetValue - (floorf(offsetValue / width) * width)) + start;
  // + start to reset back to start of original range
}

// Rotate, in Order X -> Y -> Z
void rotate(float pn[3], const float rotation[3])
{
  static float cached_rot[3] = {FLT_MAX, FLT_MAX, FLT_MAX};  // Cache for rotation values
  static float sin_rot[3], cos_rot[3];           // Cached trig values
  static float out[3];                           // Static to avoid allocation

  // Check if rotation values changed and update cache
  if (cached_rot[0] != rotation[0] || 
      cached_rot[1] != rotation[1] || 
      cached_rot[2] != rotation[2]) {
    cached_rot[0] = rotation[0];
    cached_rot[1] = rotation[1];
    cached_rot[2] = rotation[2];

    // Pre-calculate sin/cos values
    for (int i = 0; i < 3; i++) {
      float rot_rad = rotation[i] * DEG_TO_RAD;
      sin_rot[i] = sinf(rot_rad);
      cos_rot[i] = cosf(rot_rad);
    }
  }

  // X Rotation
  out[0] = pn[0];
  out[1] = pn[1] * cos_rot[0] - pn[2] * sin_rot[0];
  out[2] = pn[1] * sin_rot[0] + pn[2] * cos_rot[0];
  pn[0] = out[0]; pn[1] = out[1]; pn[2] = out[2];

  // Y Rotation
  out[0] = pn[0] * cos_rot[1] + pn[2] * sin_rot[1];
  out[1] = pn[1];
  out[2] = -pn[0] * sin_rot[1] + pn[2] * cos_rot[1];
  pn[0] = out[0]; pn[1] = out[1]; pn[2] = out[2];

  // Z Rotation
  out[0] = pn[0] * cos_rot[2] - pn[1] * sin_rot[2];
  out[1] = pn[0] * sin_rot[2] + pn[1] * cos_rot[2];
  out[2] = pn[2];
  pn[0] = out[0]; pn[1] = out[1]; pn[2] = out[2];
}

void readSensors(axis_t& tacc, axis_t& tgyr, axis_t& tmag, bool& accValid, bool& gyrValid, bool& magValid) {
#if defined(HAS_LSM9DS1)
    if (IMU.accelerationAvailable()) {
      IMU.readRawAccel(tacc.x, tacc.y, tacc.z);
      tacc.x *= -1.0f;  // Flip X
      accValid = true;
    }
    if (IMU.magneticFieldAvailable()) {
      IMU.readRawMagnet(tmag.x, tmag.y, tmag.z);
      magValid = true;
    }
    if (IMU.gyroscopeAvailable()) {
      IMU.readRawGyro(tgyr.x, tgyr.y, tgyr.z);
      tgyr.x *= -1.0f;  // Flip X to match other sensors
      gyrValid = true;
    }
#endif

#if defined(HAS_BMI270)
    int8_t rslt;
    uint16_t int_status = 0;
    struct bmi2_sens_data sensor_data = {{0}};
    rslt = bmi2_get_int_status(&int_status, &bmi2_dev);
    bmi2_error_codes_print_result(rslt);
    /* To check the data ready interrupt status and print the status for 10 samples. */
    if ((int_status & BMI2_ACC_DRDY_INT_MASK) && (int_status & BMI2_GYR_DRDY_INT_MASK)) {
      /* Get accel and gyro data for x, y and z axis. */
      rslt = bmi2_get_sensor_data(&sensor_data, &bmi2_dev);
      bmi2_error_codes_print_result(rslt);

      /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
      tacc.x = lsb_to_mps2(sensor_data.acc.y, 2, bmi2_dev.resolution) / GRAVITY_EARTH;
      tacc.y = -1.0f * lsb_to_mps2(sensor_data.acc.x, 2, bmi2_dev.resolution) / GRAVITY_EARTH;
      tacc.z = lsb_to_mps2(sensor_data.acc.z, 2, bmi2_dev.resolution) / GRAVITY_EARTH;
      // printk("\nAccX=%4.2f,Y=%4.2f,Z=%4.2f\n", tacc.x, tacc.y, tacc.z);
      /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */

      tgyr.x = lsb_to_dps(sensor_data.gyr.y, 2000, bmi2_dev.resolution);
      tgyr.y = -1.0f * lsb_to_dps(sensor_data.gyr.x, 2000, bmi2_dev.resolution);
      tgyr.z = lsb_to_dps(sensor_data.gyr.z, 2000, bmi2_dev.resolution);
      // printk("GyrX=%4.2f,Y=%4.2f,Z=%4.2f\n", tgyr.x, tgyr.y, tgyr.z);
      accValid = true;
      gyrValid = true;
    }
#endif

#if defined(HAS_BMM150)
    if(hasMag) {
      int8_t rbslt;
      uint8_t data_ready = 0;
      rslt = bmm150_get_regs(BMM150_REG_DATA_READY_STATUS, &data_ready, 1, &bmm1_dev);
      if(rslt == BMM150_OK) {
        if(data_ready & 0x01) {
          struct bmm150_mag_data mag_data;
          rbslt = bmm150_read_mag_data(&mag_data, &bmm1_dev);
          if (rbslt != BMM150_OK) {
            bmm150_error_codes_print_result("bmm150_read_mag_data", rbslt);
          } else {
            tmag.x = mag_data.y;
            tmag.y = mag_data.x;
            tmag.z = mag_data.z;
            magValid = true;
          }
        }
      } else {
        bmm150_error_codes_print_result("bmm150_get_regs", rslt);
      }
    }
#endif

#if defined(HAS_LSM6DS3)
    int16_t data_raw_acceleration[3];
    int16_t data_raw_angular_rate[3];
    lsm6ds3tr_c_reg_t reg;
    lsm6ds3tr_c_status_reg_get(&dev_ctx, &reg.status_reg);
    if (reg.status_reg.xlda) {
      /* Read acceleration data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      tacc.x = (float)lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[0]) / 1000.0f;
      tacc.y = (float)lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[1]) / 1000.0f;
      tacc.z = (float)lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[2]) / 1000.0f;
      accValid = true;
    }
    if (reg.status_reg.gda) {
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      lsm6ds3tr_c_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      tgyr.x = (float)lsm6ds3tr_c_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) / 1000.0f;
      tgyr.y = (float)lsm6ds3tr_c_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) / 1000.0f;
      tgyr.z = (float)lsm6ds3tr_c_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) / 1000.0f;
      gyrValid = true;
    }

#endif

#if defined(HAS_QMC5883)
    if(hasMag) {
      if (qmc5883Read(tmag.data)) {
        magValid = true;
      }
    }
#endif

#if defined(HAS_MPU6500)
    // Read MPU6500
    short _gyro[3];
    short _accel[3];
    if (!mpu_get_accel_reg(_accel, nullptr)) accValid = true;
    unsigned short ascale = 1;
    mpu_get_accel_sens(&ascale);
    tacc.x = (float)_accel[0] / (float)ascale;
    tacc.y = (float)_accel[1] / (float)ascale;
    tacc.z = (float)_accel[2] / (float)ascale;
    if (!mpu_get_gyro_reg(_gyro, nullptr)) gyrValid = true;
    float gscale = 1.0f;
    mpu_get_gyro_sens(&gscale);
    tgyr.x = _gyro[0] / gscale;
    tgyr.y = _gyro[1] / gscale;
    tgyr.z = _gyro[2] / gscale;
#endif

#if defined(HAS_MPU6886)
    if(!mpu6886.getAccelData(&tacc.x, &tacc.y, &tacc.z))
      accValid = true;
    if(!mpu6886.getGyroData(&tgyr.x, &tgyr.y, &tgyr.z)) {
      gyrValid = true;
    }
#endif
}

/* reset_fusion()
 *      Causes the madgwick filter to reset. Used when board rotation changes
 */

void reset_fusion()
{
  // TODO add a mutex here.
  madgreads = 0;
  madgsensbits = 0;
  firstrun = true;
  aacc = (axis_t){{0, 0, 0}};
  amag = (axis_t){{0, 0, 0}};
  LOG_INF("Resetting fusion algorithm");
}
