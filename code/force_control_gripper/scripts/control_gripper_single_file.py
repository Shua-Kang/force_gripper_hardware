#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Standalone gripper control module without ROS dependency.

This module provides interactive command-line control for the force-controlled gripper:
- Uses force_gripper.utils to find serial ports
- Reads gripper state from CH340 serial port (if available)
- Sends commands via main gripper serial port
- Interactive command-line interface for PWM, position, and initialization commands

Usage:
  python control_gripper_single_file.py

Commands (enter interactively):
  init                    Initialize gripper
  open                    Open gripper fully
  pos <left> <right>      Set position (0.0-1.0 for each finger)
  pwm <left> <right>      Set PWM speed (-1.0 to 1.0 for each finger)
  state                   Get current gripper state
  help                    Show this help
  quit                    Exit

Examples:
  init
  open
  pos 0.5 0.5
  pwm 0.3 -0.3
  state
  quit
"""

import serial
import threading
import json
import sys
import time

# Try to import the user's utility library
try:
    import force_gripper.utils
    print("Successfully imported 'force_gripper.utils'.")
except ImportError as e:
    print(f"Error importing force_gripper.utils: {e}")
    print("Please ensure force_gripper is properly installed.")
    sys.exit(1)

# --- Global variables ---
cmd_port_name = None
stat_port_name = None
cmd_ser = None  # Command serial port
stat_ser = None  # Status serial port

running = True  # Control thread flag
lock = threading.Lock()  # Protect cmd_ser.write access

# --- 1. Status reader thread ---

def status_reader_thread():
    """
    Background thread to read gripper status from CH340 port.
    """
    global stat_ser, running

    print("Status reader thread started.")

    while running:
        if stat_ser is None:
            time.sleep(0.5)
            continue

        try:
            # Read a line of data (Arduino println ends with '\n')
            line = stat_ser.readline()

            if not line:
                continue

            line = line.strip()

            # Simple packet validation
            if not line.startswith(b'<') or not line.endswith(b'>'):
                if line:  # Print non-empty invalid data
                    print(f"Warning: Received malformed status: {line}")
                continue

            # Parse data: <id1, pos_n1, tgt_n1, pwm_n1, id2, pos_n2, tgt_n2, pwm_n2>
            parts = line[1:-1].split(b',')
            if len(parts) != 8:
                print(f"Warning: Received incomplete status packet: {line}")
                continue

            # Convert all parts to float
            data = [float(p) for p in parts]

            # Format and display status
            status_str = (f"State - Motor1(ID{int(data[0])}): pos={data[1]:.3f}, "
                         f"tgt={data[2]:.3f}, pwm={data[3]:.3f} | "
                         f"Motor2(ID{int(data[4])}): pos={data[5]:.3f}, "
                         f"tgt={data[6]:.3f}, pwm={data[7]:.3f}")
            print(f"[{time.strftime('%H:%M:%S')}] {status_str}")

        except serial.SerialException as e:
            print(f"Serial error in status thread: {e}")
            if running:
                print("Status port error. Thread stopping.")
                running = False
            break
        except IOError as e:
            if not running:  # Ignore IO errors during normal shutdown
                break
            print(f"IO error in status thread: {e}")
            break
        except ValueError as e:
            print(f"Warning: Value error parsing status (likely bad data): {e}")
        except Exception as e:
            print(f"Error in status thread: {e}")

    print("\nStatus reader thread stopping.")

# --- 2. Command functions ---

def send_command(command, value=None):
    """
    Send command to gripper via serial port.
    """
    global cmd_ser, running, lock

    if cmd_ser is None or not cmd_ser.is_open or not running:
        print("Warning: Serial port not ready.")
        return

    arduino_cmd = None

    # Convert command format
    try:
        if command == 'init':
            arduino_cmd = '<"initialization">'
        elif command == 'open':
            arduino_cmd = '<"open">'
        elif command == 'pos' and isinstance(value, list) and len(value) == 2:
            p1 = float(value[0])
            p2 = float(value[1])
            arduino_cmd = '<"pos", {:.4f}, {:.4f}>'.format(p1, p2)
        elif command == 'pwm' and isinstance(value, list) and len(value) == 2:
            v1 = float(value[0])
            v2 = float(value[1])
            arduino_cmd = '<"pwm", {:.4f}, {:.4f}>'.format(v1, v2)
        else:
            print(f"Error: Unknown or malformed command: {command}, {value}")
            return
    except (TypeError, ValueError) as e:
        print(f"Error converting command values: {e}")
        return

    # Send command
    if arduino_cmd:
        try:
            with lock:  # Use lock for atomic write
                print(f"\nSending: {arduino_cmd}")
                cmd_ser.write((arduino_cmd + '\n').encode('utf-8'))
        except serial.SerialException as e:
            print(f"Failed to send command: {e}")
        except Exception as e:
            print(f"Error writing to serial port: {e}")

# --- 3. Interactive command interface ---

def print_usage():
    """Print usage instructions"""
    print("\n" + "="*60)
    print("           Gripper Interactive Controller")
    print("="*60)
    print("Available commands:")
    print("  init                    Initialize gripper")
    print("  open                    Open gripper fully")
    print("  pos <left> <right>      Set position (0.0-1.0 for each finger)")
    print("  pwm <left> <right>      Set PWM speed (-1.0 to 1.0 for each finger)")
    print("  state                   Get current gripper state")
    print("  help                    Show this help")
    print("  quit                    Exit")
    print("\nExamples:")
    print("  init")
    print("  open")
    print("  pos 0.5 0.5")
    print("  pwm 0.3 -0.3")
    print("  state")
    print("="*60)

def parse_command(command_line):
    """Parse interactive command input"""
    parts = command_line.strip().split()
    if not parts:
        return None, None

    command = parts[0].lower()

    if command in ['init', 'open', 'state', 'help', 'quit']:
        return command, None
    elif command in ['pos', 'pwm']:
        if len(parts) != 3:
            print(f"Error: '{command}' command requires 2 arguments")
            return None, None
        try:
            val1 = float(parts[1])
            val2 = float(parts[2])
            return command, [val1, val2]
        except ValueError:
            print(f"Error: Arguments must be numbers for '{command}' command")
            return None, None
    else:
        print(f"Error: Unknown command '{command}'")
        return None, None

def main():
    """Main function"""
    global cmd_port_name, stat_port_name, cmd_ser, stat_ser, running

    print("Starting standalone gripper controller...")

    # --- 1. Find ports ---
    try:
        cmd_port_name = force_gripper.utils.find_port_by_name("gripper")
        try:
            stat_port_name = force_gripper.utils.find_port_by_name("gripper_ch340")
            print(f"Found status port: {stat_port_name}")
        except Exception as e:
            stat_port_name = None
            print(f"Status port not found (optional): {e}")

        if not cmd_port_name:
            print("Error: Could not find gripper command port. Exiting.")
            return

        print(f"Found command port: {cmd_port_name}")

    except Exception as e:
        print(f"Error finding ports: {e}")
        return

    # --- 2. Open serial ports ---
    try:
        # Command port
        cmd_ser = serial.Serial(
            port=cmd_port_name,
            baudrate=115200,
            timeout=1.0,
            write_timeout=1.0
        )
        print(f"Opened command port: {cmd_port_name}")

        # Status port (optional)
        if stat_port_name:
            stat_ser = serial.Serial(
                port=stat_port_name,
                baudrate=115200,
                timeout=1.0
            )
            print(f"Opened status port: {stat_port_name}")
        else:
            print("No status port available - state will show as null")

    except serial.SerialException as e:
        print(f"Error opening serial ports: {e}")
        return

    # --- 3. Start status reader thread ---
    if stat_ser:
        status_thread = threading.Thread(target=status_reader_thread, daemon=True)
        status_thread.start()
        print("Status monitoring started.")
    else:
        print("Running without status feedback.")

    # --- 4. Interactive command loop ---
    print_usage()
    print("Enter commands (type 'help' for help, 'quit' to exit):")

    try:
        while running:
            # Get user input
            try:
                command_line = input("gripper> ").strip()
            except EOFError:
                # Handle Ctrl+D
                print("\nReceived EOF. Exiting...")
                break

            if not command_line:
                continue

            # Parse command
            command, value = parse_command(command_line)

            if command is None:
                continue

            if command == 'quit':
                print("Exiting...")
                break
            elif command == 'help':
                print_usage()
            elif command == 'state':
                # For state command, just show current status
                # The status thread already prints continuously
                print("Status monitoring is active (see above)")
                if not stat_ser:
                    print("State: null (no status port available)")
            else:
                # Send command
                send_command(command, value)

    except KeyboardInterrupt:
        print("\nInterrupted by user (Ctrl+C)")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # --- Cleanup ---
        print("\nShutting down...")

        running = False  # Stop all threads
        time.sleep(0.2)  # Wait for threads to notice

        try:
            if cmd_ser and cmd_ser.is_open:
                print(f"Closing command port: {cmd_port_name}")
                cmd_ser.close()
        except Exception as e:
            print(f"Error closing command port: {e}")

        try:
            if stat_ser and stat_ser.is_open:
                print(f"Closing status port: {stat_port_name}")
                stat_ser.close()
        except Exception as e:
            print(f"Error closing status port: {e}")

        print("Gripper controller shutdown complete.")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Startup error: {e}")
        sys.exit(1)