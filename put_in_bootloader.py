#!/usr/bin/env python3

import serial
import time
import sys

def put_in_bootloader(port_name):
    """
    Put HeadTracker into bootloader mode using a custom command.

    Args:
        port_name (str): Serial port name
        
    Returns:
        bool: True if successful, False otherwise
    """
    
    try:
        # Custom bootloader command (HeadTracker firmware specific)
        try:
            port = serial.Serial(port_name, 1200, timeout=1)
            port.rts = True
            port.dtr = False
            bootcmd = b'\x02{"Cmd":"Boot"}\xDA\x0c\x03\r\n'
            port.write(bootcmd)
            port.flush()
            port.close()
            time.sleep(1)
        except:
            pass
        
        return True
        
    except serial.SerialException as e:
        print(f"Error: Could not access port {port_name}: {e}")
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python put_in_bootloader.py <port_name>")
        print("Example: python put_in_bootloader.py /dev/cu.usbmodem2101")
        sys.exit(1)
    
    port_name = sys.argv[1]
    
    if put_in_bootloader(port_name):
        print("Success! Board should be in bootloader mode.")
    else:
        print("Failed to put board into bootloader mode.")
        sys.exit(1)

if __name__ == "__main__":
    main()
