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

#define CRSF_ACTUAL_RATE_CHANNEL 12

// #define DEBUG_SENSOR_RATES

typedef union {
  struct {
    float x, y, z;
  };
  float data[3];
} axis_t;

void gyroCalibrate();
void detectDoubleTap();

inline float magnitude(const axis_t& v) {
  return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline uint16_t clamp_channel(float value, uint16_t min_val, uint16_t max_val) {
  return MAX(MIN((uint16_t)value, max_val), min_val);
}

static float auxdata[10];
static axis_t racc = {{0, 0, 0}};
static axis_t rmag = {{0, 0, 0}};
static axis_t rgyr = {{0, 0, 0}};
static axis_t acc = {{0, 0, 0}};
static axis_t mag = {{0, 0, 0}};
static axis_t gyr = {{0, 0, 0}};
static uint16_t rollout_ui_shared = 0;
static bool trpOutputEnabled = false;  // Default to disabled T/R/P output
static bool gyroCalibrated = false;

// Input Channel Data
static uint16_t ppm_in_chans[16];
static uint16_t uart_in_chans[16];
static uint16_t bt_chans[TrackerSettings::BT_CHANNELS];
static float bt_chansf[TrackerSettings::BT_CHANNELS];

// Output channel data
static uint16_t channel_data[16];

Madgwick madgwick;

int64_t usduration = 0; //TODO unsinged
int64_t senseUsDuration = 0;

const struct device *i2c_dev = nullptr;

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

static bool butdwn = false;
static int madgreads = 0;
static uint8_t madgsensbits = 0;
static volatile bool firstrun = true;
static axis_t aacc = {{0, 0, 0}};
static axis_t amag = {{0, 0, 0}};

// Analog Filters
SF1eFilter *anFilter[AN_CH_CNT];

// Battery Voltage Filter
SF1eFilter *anVoltFilter;

static struct k_poll_signal senseThreadRunSignal = K_POLL_SIGNAL_INITIALIZER(senseThreadRunSignal);
struct k_poll_event senseRunEvents[1] = {
    K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &senseThreadRunSignal),
};

static struct k_poll_signal calculateThreadRunSignal =
    K_POLL_SIGNAL_INITIALIZER(calculateThreadRunSignal);
struct k_poll_event calculateRunEvents[1] = {
    K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
                             &calculateThreadRunSignal),
};

int sense_Init()
{
  // If an I2C bus is defined in the device tree, initialize it
#if DT_NODE_EXISTS(DT_ALIAS(i2csensor))
  const struct device *i2c_dev = DEVICE_DT_GET(DT_ALIAS(i2csensor));
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
    LOG_ERR("Failed to initalize LSM9DS1 Sensor");
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
    LOG_ERR("Failed to initalize BMI270");
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
  // Initalize Gesture Sensor
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

  // Start reading the IMU sensors + fusion algorithm
  k_poll_signal_raise(&senseThreadRunSignal, 1);
  LOG_INF("Sense Thread Signal Raised");

  // Start doing the other calculations
  k_poll_signal_raise(&calculateThreadRunSignal, 1);
  LOG_INF("Calculate Thread Signal Raised");

  return 0;
}

//----------------------------------------------------------------------
// Calculations and Main Channel Thread
//----------------------------------------------------------------------

void calculate_Thread()
{
  LOG_INF("Calculate Thread Loaded");
  while (1) {
    // Do not execute below until after initialization has happened
    k_poll(calculateRunEvents, 1, K_FOREVER);

    if (k_sem_count_get(&flashWriteSemaphore) == 1) {
      k_msleep(10);
      continue;
    }

    usduration = micros64();

    // Toggles output on and off if long pressed
    bool butlngdwn = false;
    if (wasButtonLongPressed()) {
      LOG_INF("Reset Center Long Pressed");
      trpOutputEnabled = !trpOutputEnabled;
      butlngdwn = true;
    }

    static bool btbtnlngupdated = false;
    if (BTGetMode() == BTPARARMT) {
      if (butlngdwn && btbtnlngupdated == false) {
        BTRmtSendButtonPress(true);  // Send the long press over bluetooth to remote board
        btbtnlngupdated = true;
      } else if (btbtnlngupdated == true) {
        btbtnlngupdated = false;
      }
    }


    // Reset on tilt
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
      if (rollout_ui_shared == trkset.getRll_Max()) {
        if (tiltpeak == false && minmax == HITNONE) {
          tiltpeak = true;
          minmax = HITMAX;
        } else if (minmax == HITMIN) {
          minmax = HITNONE;
          tiltpeak = false;
          doresetontilt = true;
        }

      } else if (rollout_ui_shared == trkset.getRll_Min()) {
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

    /* ************************************************************
     *       Build channel data
     *
     * Build Channel Data
     *   1) Reset all channels to disabled
     *   2) Set PPMin channels
     *   3) Set SBUSin channels
     *   4) Set received BT channels
     *   5) Reset Center on PPM channel
     *   6) Set auxiliary functions
     *   7) Set analog channels
     *   8) Set Reset Center pulse channel
     *   9) Override desired channels with pan/tilt/roll (now handled in sensor thread)
     *  10) Sync channel outputs between calculate and sensor thread
     *  11) Output to PPMout
     *  12) Output to Bluetooth
     *  13) Output PWM channels
     *  14) Output to USB Joystick
     *
     *  Channels should all be set to zero if they don't have valid data
     *  Only on the output should a channel be set to center if it's still zero
     *  Allows the GUI to know which channels are valid
     */

    // 1) Reset all Channels to zero which means they have no data
    static uint16_t local_channel_data[16];
    for (int i = 0; i < 16; i++) local_channel_data[i] = 0;

    // 2) Read all PPM inputs
    PpmIn_execute();
    for (int i = 0; i < 16; i++)
      ppm_in_chans[i] = 0;  // Reset all PPM in channels to Zero (Not active)
    int ppm_in_chcnt = PpmIn_getChannels(ppm_in_chans);
    if (ppm_in_chcnt >= 4 && ppm_in_chcnt <= 16) {
      for (int i = 0; i < MIN(ppm_in_chcnt, 16); i++) {
        local_channel_data[i] = ppm_in_chans[i];
      }
    }

    // 3) Set all incoming UART values (Sbus/Crsf)
    bool isUartValid = UartGetChannels(uart_in_chans);
    static bool lostmsgsent = false;
    static bool recmsgsent = false;
    if (!isUartValid) {
      if (!lostmsgsent) {
        LOG_ERR("Uart(SBUS/CRSF) Data Lost");
        lostmsgsent = true;
      }
      recmsgsent = false;
      // SBUS data still valid, set the channel values to the last SBUS
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

    // 4) Set all incoming BT values
    // Bluetooth cannot send a zero value for a channel with PARA. Radios see this as invalid data.
    // So, if the data is coming from a BLE head unit it also has a characteristic to nofity which
    // ones are valid alloww PPM/SBUS pass through on the head or remote boards on ch 1-8
    // If the data is coming from a PARA radio all 8ch's are going to have values, all PPM/SBUS
    // inputs 1-8 will be overridden

    for (int i = 0; i < TrackerSettings::BT_CHANNELS; i++)
      bt_chans[i] = 0;  // Reset all BT in channels to Zero (Not active)
    for (int i = 0; i < TrackerSettings::BT_CHANNELS; i++) {
      uint16_t btvalue = BTGetChannel(i);
      if (btvalue > 0) {
        bt_chans[i] = btvalue;
        local_channel_data[i] = btvalue;
      }
    }

    // 5) If selected input channel went > 1800us reset the center
    // wait for it to drop below 1700 before allowing another reset
    /*int rstppmch = trkset.resetCntPPM() - 1;
    static bool hasrstppm=false;
    if(rstppmch >= 0 && rstppmch < 16) {
        if(local_channel_data[rstppmch] > 1800 && hasrstppm == false) {
            LOG_INF("Reset Center - Input Channel %d > 1800us", rstppmch+1);
            pressButton();
            hasrstppm = true;
        } else if (local_channel_data[rstppmch] < 1700 && hasrstppm == true) {
            hasrstppm = false;
        }
    }*/ //REMOVED as of V2.1

    // 6) Set Auxiliary Functions
    int aux0ch = trkset.getAux0Ch();
    int aux1ch = trkset.getAux1Ch();
    int aux2ch = trkset.getAux2Ch();
    if (aux0ch > 0 || aux1ch > 0 || aux2ch > 0) {
      buildAuxData();
      if (aux0ch > 0) local_channel_data[aux0ch - 1] = auxdata[trkset.getAux0Func()];
      if (aux1ch > 0) local_channel_data[aux1ch - 1] = auxdata[trkset.getAux1Func()];
      if (aux2ch > 0) local_channel_data[aux2ch - 1] = auxdata[trkset.getAux2Func()];
    }

    // 7) Set Analog Channels
    // Battery voltage monitor is always analog zero if it has the feature
#ifdef ANVOLTMON
    float anbatt = SF1eFilterDo(anVoltFilter, analogRead(ANVOLTMON));
    anbatt *= ANVOLTMON_SCALE;
    anbatt += ANVOLTMON_OFFSET;
#endif

#ifdef AN0
    if (trkset.getAn0Ch() > 0) {
      float an4 = SF1eFilterDo(anFilter[0], analogRead(AN0));
      an4 *= trkset.getAn0Gain();
      an4 += trkset.getAn0Off();
      an4 += TrackerSettings::MIN_PWM;
      an4 = MAX(TrackerSettings::MIN_PWM, MIN(TrackerSettings::MAX_PWM, an4));
      local_channel_data[trkset.getAn0Ch() - 1] = an4;
    }
#endif
#ifdef AN1
    if (trkset.getAn1Ch() > 0) {
      float an5 = SF1eFilterDo(anFilter[1], analogRead(AN1));
      an5 *= trkset.getAn1Gain();
      an5 += trkset.getAn1Off();
      an5 += TrackerSettings::MIN_PWM;
      an5 = MAX(TrackerSettings::MIN_PWM, MIN(TrackerSettings::MAX_PWM, an5));
      local_channel_data[trkset.getAn1Ch() - 1] = an5;
    }
#endif
#ifdef AN2
    if (trkset.getAn2Ch() > 0) {
      float an6 = SF1eFilterDo(anFilter[2], analogRead(AN2));
      an6 *= trkset.getAn2Gain();
      an6 += trkset.getAn2Off();
      an6 += TrackerSettings::MIN_PWM;
      an6 = MAX(TrackerSettings::MIN_PWM, MIN(TrackerSettings::MAX_PWM, an6));
      local_channel_data[trkset.getAn2Ch() - 1] = an6;
    }
#endif
#ifdef AN3
    if (trkset.getAn3Ch() > 0) {
      float an7 = SF1eFilterDo(anFilter[3], analogRead(AN3));
      an7 *= trkset.getAn3Gain();
      an7 += trkset.getAn3Off();
      an7 += TrackerSettings::MIN_PWM;
      an7 = MAX(TrackerSettings::MIN_PWM, MIN(TrackerSettings::MAX_PWM, an7));
      local_channel_data[trkset.getAn3Ch() - 1] = an7;
    }
#endif

    // 8) First decide if 'reset center' pulse should be sent
    bool initiatereset;
    k_sched_lock();
    if (butdwn) {
      butdwn = false;
      initiatereset = true;
    } else {
      initiatereset = false;
    }
    k_sched_unlock();

    // If button was pressed and this is a remote bluetooth boart send the button press back
    static bool btbtnupdated = false;
    if (BTGetMode() == BTPARARMT) {
      if (initiatereset && btbtnupdated == false) {
        BTRmtSendButtonPress(false);  // Send the short press over bluetooth to remote board
        btbtnupdated = true;
      } else if (btbtnupdated == true) {
        btbtnupdated = false;
      }
    }

    static float pulsetimer = 0;
    static bool sendingresetpulse = false;
    int alertch = trkset.getAlertCh();
    if (alertch > 0) {
      // Indicate gyro calibration and centering status
      if (sendingresetpulse) {
        local_channel_data[alertch - 1] = gyroCalibrated ? TrackerSettings::MAX_PWM : TrackerSettings::DEF_MAX_PWM;
      } else {
        local_channel_data[alertch - 1] = gyroCalibrated ? TrackerSettings::DEF_MIN_PWM : TrackerSettings::MIN_PWM;
      }
      if (initiatereset) {
        sendingresetpulse = true;
        pulsetimer = 0.0f;
      }
      if (sendingresetpulse) {
        pulsetimer += (float)CALCULATE_PERIOD / 1000000.0f;
        if (pulsetimer > TrackerSettings::RECENTER_PULSE_DURATION) {
          sendingresetpulse = false;
        }
      }
    }

    // 9) Tilt/Roll/Pan is now set in senor thread, this section handles only TRP output enable/disable

    // If the long press for enable/disable isn't set or if there is no reset button configured
    //   always enable the T/R/P outputs
    static bool lastbutmode = false;
    bool buttonpresmode = trkset.getButLngPs();
    if (buttonpresmode == false) trpOutputEnabled = true;

    // On user enabling the button press mode in the GUI default to TRP output off.
    if (lastbutmode == false && buttonpresmode == true) {
      trpOutputEnabled = false;
    }
    lastbutmode = buttonpresmode;

    // If uart output set to CRSF_OUT, force channel 5 (AUX1/ARM) to high, will override all other
    // channels
    if (trkset.getUartMode() == TrackerSettings::UART_MODE_CRSFOUT) {
      if (trkset.getCh5Arm()) local_channel_data[4] = 2000;
    }

    // 10 Sync channel outputs between calculate and sensor thread
    int tlti = trkset.getTltCh()-1;
    int rlli = trkset.getRllCh()-1;
    int pani = trkset.getPanCh()-1;

    k_sched_lock();
    if (tlti >= 0 && tlti < 16) local_channel_data[tlti] = channel_data[tlti];
    if (rlli >= 0 && rlli < 16) local_channel_data[rlli] = channel_data[rlli];
    if (pani >= 0 && pani < 16) local_channel_data[pani] = channel_data[pani];
    local_channel_data[CRSF_ACTUAL_RATE_CHANNEL-1] = channel_data[CRSF_ACTUAL_RATE_CHANNEL-1];

    for (int i = 0; i < 16; i++) {
      if (i == tlti || i == rlli || i == pani || i == CRSF_ACTUAL_RATE_CHANNEL - 1) {
        continue;
      } else {
        if (local_channel_data[i] == 0) {
          // channel_data will be sent to UART in sensor thread, it needs to be centered
          channel_data[i] = TrackerSettings::PPM_CENTER;
        } else {
          channel_data[i] = local_channel_data[i];
        }
      }
    }
    k_sched_unlock();

    // 11) Set the PPM Outputs
    PpmOut_execute();
    for (int i = 0; i < PpmOut_getChnCount(); i++) {
      uint16_t ppmout = local_channel_data[i];
      if (ppmout == 0) ppmout = TrackerSettings::PPM_CENTER;
      PpmOut_setChannel(i, ppmout);
    }

    // 12) Set all the BT Channels, send the zeros don't center
    bool bleconnected = BTGetConnected();
    trkset.setDataBtAddr(BTGetAddress());
    for (int i = 0; i < TrackerSettings::BT_CHANNELS; i++) {
      BTSetChannel(i, local_channel_data[i]);
    }

    // 13) Set PWM Channels
    int8_t pwmchs[4] = {trkset.getPwm0(), trkset.getPwm1(), trkset.getPwm2(), trkset.getPwm3()};
    for (int i = 0; i < 4; i++) {
      int pwmch = pwmchs[i] - 1;
      if (pwmch >= 0 && pwmch < 16) {
        uint16_t pwmout = local_channel_data[pwmch];
        if (pwmout == 0) pwmout = TrackerSettings::PPM_CENTER;
        setPWMValue(i, pwmout);
      }
    }

    // 14 Set USB Joystick Channels, Only 8 channels, Half rate or USB is overwhelmed
    static uint32_t joystick_update = 0;
    if(joystick_update++ > 1) {
      joystick_update = 0;
      set_JoystickChannels(local_channel_data);
    }

    // Update the settings for the GUI
    // Serial also uses this data, make sure writes are complete.
    //  If data thread has it locked just skip this reading
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

      // PPM Input Values
      trkset.setDataPpmCh(ppm_in_chans);
      trkset.setDataBtCh(bt_chans);
      trkset.setDataUartCh(uart_in_chans);
      trkset.setDataChOut(local_channel_data);

      trkset.setDataTrpEnabled(trpOutputEnabled);
      trkset.setDataGyroCal(gyroCalibrated);

      // Qauterion Data
      float *qd = madgwick.getQuat();
      trkset.setDataQuat(qd);

      // Bluetooth connected
      trkset.setDataBtCon(bleconnected);
      k_mutex_unlock(&data_mutex);
    }

    // Adjust sleep for a more accurate period
    usduration = micros64() - usduration;
    if (CALCULATE_PERIOD - usduration <
        CALCULATE_PERIOD * 0.7) {  // Took a long time. Will crash if sleep is too short
      LOG_ERR("Calculate Thread Overrun %lld", usduration);
      k_usleep(CALCULATE_PERIOD);
    } else {
      k_usleep(CALCULATE_PERIOD - usduration);
    }

#if defined(DEBUG_SENSOR_RATES)
    static int mcount = 0;
    static int64_t mmic = millis64() + 1000;
    if (mmic < millis64()) {  // Every Second
      mmic = millis64() + 1000;
      LOG_INF("Calc Rate = %d", mcount);
      mcount = 0;
    }
    mcount++;
#endif
  }
}

//----------------------------------------------------------------------
// Sensor Reading Thread
//----------------------------------------------------------------------

void sensor_Thread()
{
  LOG_INF("Sensor Thread Loaded");
  while (1) {
    // Do not execute below until after initialization has happened
    k_poll(senseRunEvents, 1, K_FOREVER);

    if (k_sem_count_get(&flashWriteSemaphore) == 1) {
      k_msleep(10);
      continue;
    }

    senseUsDuration = micros64();

#if defined(HAS_APDS9960)
    // Reset Center on Proximity, Don't need to update this often
    static int sensecount = 0;
    static int minproximity = 100;  // Keeps smallest proximity read.
    static int maxproximity = 0;    // Keeps largest proximity value read.
    if (blesenseboard && sensecount++ >= 10) {
      sensecount = 0;
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

    // Setup Rotations
    float rotation[3] = {trkset.getRotX(), trkset.getRotY(), trkset.getRotZ()};

    // Read the data from the sensors
    axis_t tacc = {{0.0f, 0.0f, 0.0f}}, tgyr = {{0.0f, 0.0f, 0.0f}}, tmag = {{0.0f, 0.0f, 0.0f}};
    bool accValid = false;
    bool gyrValid = false;
    bool magValid = false;

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
      /* Read magnetic field data */
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

        // For inital orientation setup
        madgsensbits |= MADGINIT_MAG;
      }
    } else {
      mag = (axis_t){{0, 0, 0}};
      madgsensbits |= MADGINIT_MAG;
    }

    // Run Gyro Calibration, only on good gyro data
    if (gyrValid) {
      gyroCalibrate();
      // If double tap detection is enabled, check for it
      // if(trkset.getRstOnDbltTap())
      //   detectDoubleTap();
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

    if (wasButtonPressed()) {
      LOG_INF("Reset Center Short Pressed");
      rolloffset = roll;
      panoffset = pan;
      tiltoffset = tilt;
      butdwn = true;
    }

    // Fast CRSF mode - send CRSF directly from sensor thread for minimum latency
    // Calculate outputs
    float tiltout = (tilt - tiltoffset) * trkset.getTlt_Gain() * (trkset.isTiltReversed() ? -1.0f : 1.0f);
    float rollout = (roll - rolloffset) * trkset.getRll_Gain() * (trkset.isRollReversed() ? -1.0f : 1.0f);
    float panout = normalize((pan - panoffset), -180, 180) * trkset.getPan_Gain() * (trkset.isPanReversed() ? -1.0f : 1.0f);

    // Convert to channel values
    tiltout_ui = clamp_channel(tiltout + trkset.getTlt_Cnt(), trkset.getTlt_Min(), trkset.getTlt_Max());
    rollout_ui = clamp_channel(rollout + trkset.getRll_Cnt(), trkset.getRll_Min(), trkset.getRll_Max());
    panout_ui = clamp_channel(panout + trkset.getPan_Cnt(), trkset.getPan_Min(), trkset.getPan_Max());

    // Set head tracking channels
    int8_t tltch = trkset.getTltCh();
    int8_t rllch = trkset.getRllCh();
    int8_t panch = trkset.getPanCh();
    if (tltch > 0 && tltch <= 16) channel_data[tltch - 1] = trpOutputEnabled == true ? tiltout_ui : trkset.getTlt_Cnt();
    if (rllch > 0 && rllch <= 16) channel_data[rllch - 1] = trpOutputEnabled == true ? rollout_ui : trkset.getRll_Cnt();
    if (panch > 0 && panch <= 16) channel_data[panch - 1] = trpOutputEnabled == true ? panout_ui : trkset.getPan_Cnt();

    static uint32_t crsfActualRate = 0;
    channel_data[CRSF_ACTUAL_RATE_CHANNEL - 1] = crsfActualRate / 2 + 1500; // Show crsfActualRate in Betaflight OSD as RC channel

    uint32_t sensorPeriod = SENSOR_PERIOD;
    if (trkset.getUartMode() == TrackerSettings::UART_MODE_CRSFOUT) {
      uint32_t crsfRate = ((trkset.getCrsfTxRate()+1) * 2);
      sensorPeriod = (1.0f / (float)crsfRate) * 1.0e6f;

      k_sched_lock(); // don't allow uart thread to read while we are writing
      UartSetChannels(channel_data);
      k_sched_unlock();
    }

    // k_sched_lock(); // Not needed because of high priority of this thread
    rollout_ui_shared = rollout_ui;
    trkset.setDataTilt(tilt);
    trkset.setDataRoll(roll);
    trkset.setDataPan(pan);

    trkset.setDataTiltOff(tilt - tiltoffset);
    trkset.setDataRollOff(roll - rolloffset);
    trkset.setDataPanOff(normalize(pan - panoffset, -180, 180));

    trkset.setDataTiltOut(tiltout_ui);
    trkset.setDataRollOut(rollout_ui);
    trkset.setDataPanOut(panout_ui);
    // k_sched_unlock();

    // Adjust sleep for a more accurate period
    senseUsDuration = micros64() - senseUsDuration;
    if (sensorPeriod - senseUsDuration <
        sensorPeriod * 0.4) {  // Took a long time. Will crash if sleep is too short
      LOG_ERR("Sensor Thread Overrun %lld", senseUsDuration);
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
  static float last_acc_mag = 0;
  static uint64_t lasttaptime = 0;
  static uint64_t lasttime = 0;
  uint64_t time = millis64();
  uint64_t timediff;

  float deltatime = (float)(time - lasttime) / 1000.0f;
  if (deltatime == 0.0f) return;
  lasttime = time;

  float acc_magnitude = magnitude(racc);
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

  float gyro_magnitude = magnitude(rgyr);
  float acc_magnitude = magnitude(racc);
  float gyro_dif = (gyro_magnitude - last_gyro_mag) / deltatime;
  last_gyro_mag = gyro_magnitude;
  float acc_dif = (acc_magnitude - last_acc_mag) / deltatime;
  last_acc_mag = acc_magnitude;

  // Is Gyro anc Accelerometer stable?
  if (fabsf(gyro_dif) < GYRO_STABLE_DIFF && fabsf(acc_dif) < ACC_STABLE_DIFF) {
    // First run, preload filter
    if (filter_samples == 0) {
      filt_gyro.x = rgyr.x;
      filt_gyro.y = rgyr.y;
      filt_gyro.z = rgyr.z;
      sent_gyro_cal_msg = false;
      filter_samples++;
    } else if (filter_samples < GYRO_STABLE_SAMPLES) {
      static const float GYRO_FILTER_COEFF_OLD = 1.0f - GYRO_SAMPLE_WEIGHT;
      static const float GYRO_FILTER_COEFF_NEW = GYRO_SAMPLE_WEIGHT;
      
      filt_gyro.x = (GYRO_FILTER_COEFF_OLD * filt_gyro.x) + (GYRO_FILTER_COEFF_NEW * rgyr.x);
      filt_gyro.y = (GYRO_FILTER_COEFF_OLD * filt_gyro.y) + (GYRO_FILTER_COEFF_NEW * rgyr.y);
      filt_gyro.z = (GYRO_FILTER_COEFF_OLD * filt_gyro.z) + (GYRO_FILTER_COEFF_NEW * rgyr.z);
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
  const float width = end - start;          //
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

/* Builds data for auxiliary functions
 */

void buildAuxData()
{
  float pwmrange = (TrackerSettings::MAX_PWM - TrackerSettings::MIN_PWM);
  auxdata[TrackerSettings::AUX_GYRX] = (gyr.x / 1000) * pwmrange + TrackerSettings::PPM_CENTER;
  auxdata[TrackerSettings::AUX_GYRY] = (gyr.y / 1000) * pwmrange + TrackerSettings::PPM_CENTER;
  auxdata[TrackerSettings::AUX_GYRZ] = (gyr.z / 1000) * pwmrange + TrackerSettings::PPM_CENTER;
  auxdata[TrackerSettings::AUX_ACCELX] = (acc.x / 2.0f) * pwmrange + TrackerSettings::PPM_CENTER;
  auxdata[TrackerSettings::AUX_ACCELY] = (acc.y / 2.0f) * pwmrange + TrackerSettings::PPM_CENTER;
  auxdata[TrackerSettings::AUX_ACCELZ] = (acc.z / 1.0f) * pwmrange + TrackerSettings::PPM_CENTER;
  auxdata[TrackerSettings::AUX_ACCELZO] =
      ((acc.z - 1.0f) / 2.0f) * pwmrange + TrackerSettings::PPM_CENTER;
  auxdata[TrackerSettings::BT_RSSI] =
      static_cast<float>(BTGetRSSI()) / 127.0f * pwmrange + TrackerSettings::MIN_PWM;
}
