# HeadTracker Firmware - Quick Build Guide

## Prerequisites
- Zephyr RTOS v3.7.1 installed at `/Volumes/workspace/zephyrproject`
- Zephyr SDK 0.17.0 installed
- CMake, Ninja, Python 3.8+ available

## Zephyr Project Setup (if not already done)

If you haven't set up Zephyr yet:

```bash
# Create Zephyr workspace
cd /Volumes/workspace
west init zephyrproject
cd zephyrproject

# Checkout specific version
cd zephyr
git checkout v3.7.1
cd ..

# Install dependencies
west update
west zephyr-export
pip3 install -r zephyr/scripts/requirements.txt

# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate
pip install -r zephyr/scripts/requirements.txt
```

## Quick Build Commands

### 1. Set Environment
```bash
source /Volumes/workspace/zephyrproject/.venv/bin/activate
cd /Volumes/workspace/HeadTracker/firmware/src
export ZEPHYR_BASE=/Volumes/workspace/zephyrproject/zephyr
```

### 2. Build for DTQSYS Board
```bash
west build -p -b dtqsys_ht
```

### 3. Flash Firmware

#### Option A: Using put_in_bootloader.py (Recommended)
```bash
# Find your serial port
ls /dev/cu.usbmodem*

# Put board into bootloader mode gracefully
python3 ../../put_in_bootloader.py /dev/cu.usbmodemXXXX

# Flash the firmware using Arduino IDE bossac
/Users/pm/Library/Arduino15/packages/arduino/tools/bossac/1.9.1-arduino2/bossac --port=tty.usbmodemXXXX -e -w -R ./build/zephyr/dtqsys_ht-*.bin
```

#### Option B: Manual bootloader mode (double-tap reset)
```bash
# Put board in bootloader mode (double-tap reset button)
/Users/pm/Library/Arduino15/packages/arduino/tools/bossac/1.9.1-arduino2/bossac --port=tty.usbmodemXXXX -e -w -R ./build/zephyr/dtqsys_ht-*.bin
```

## Output Files
- `build/zephyr/dtqsys_ht-*.bin` - Binary for flashing
- `build/zephyr/dtqsys_ht-*.hex` - HEX file
- `build/zephyr/dtqsys_ht-*.elf` - Debug file

## Alternative Boards
Replace `dtqsys_ht` with:
- `arduino_nano_33_ble` - Arduino Nano 33 BLE
- `xiao_ble/nrf52840` - Seeed XIAO nRF52840
- `esp32c3_devkitm` - ESP32-C3

## Troubleshooting
- If `west` not found: source the virtual environment
- If Zephyr not found: check `ZEPHYR_BASE` path
- Build fails: `rm -rf build` and retry
- Serial port not found: check USB connection and try different ports
- Use Arduino IDE bossac: `/Users/pm/Library/Arduino15/packages/arduino/tools/bossac/1.9.1-arduino2/bossac`</content>
<parameter name="filePath">/Volumes/workspace/HeadTracker/firmware/BUILD.md