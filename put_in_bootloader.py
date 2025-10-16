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
        port = serial.Serial(port_name, 1200, timeout=1)
        port.rts = True
        port.dtr = False
        
        bootcmd = b'\x02{"Cmd":"Boot"}\xDA\x0c\x03\r\n'
        bytes_written = port.write(bootcmd)
        
        # Verify the command was actually written
        if bytes_written != len(bootcmd):
            port.close()
            raise IOError(f"Failed to write complete command")
        
        port.flush()
        
        # Give device a moment to process command before closing
        time.sleep(0.1)
        port.close()
        
        # Wait for device to reboot into bootloader
        time.sleep(1)
        
        return True
        
    except serial.SerialException as e:
        error_str = str(e).lower()
        if "no such file" in error_str or "not find" in error_str:
            print(f"Error: Device not found at {port_name}")
        elif "permission denied" in error_str:
            print(f"Error: Permission denied for {port_name}")
        elif "already open" in error_str or "in use" in error_str:
            print(f"Error: Port {port_name} is already in use")
        else:
            print(f"Error: Could not access {port_name}")
        return False
    except Exception as e:
        print(f"Error: {e}")
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