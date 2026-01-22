#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS 1 node for controlling Dynamixel gripper via dual serial ports.

This node:
1. Subscribes to /gripper/command (std_msgs/String) to receive JSON commands.
2. Converts JSON commands to Arduino format and sends via 'gripper' serial port.
3. Reads status from 'gripper_ch340' serial port in a separate thread.
4. Parses status and publishes to /gripper/state (sensor_msgs/Joy).
   - msg.buttons = [ID1, ID2]
   - msg.axes = [pos_norm_1, tgt_norm_1, pwm_norm_1, pos_norm_2, tgt_norm_2, pwm_norm_2]
"""

import rospy
import serial
import threading
import json
import time
import sys

# ROS message types
from std_msgs.msg import String
from sensor_msgs.msg import Joy # Using Joy to publish status

# Try to import user's utility library
import force_gripper.utils
rospy.loginfo("Successfully imported 'force_gripper.utils'.")

# --- Global variables ---
cmd_port_name = None
stat_port_name = None
cmd_ser = None  # Command serial port object
stat_ser = None # Status serial port object

state_publisher = None

running = True  # Global flag to control threads
lock = threading.Lock() # Protect access to cmd_ser.write

# --- 1. Status reader thread ---

def status_reader_thread():
    """
    Run in separate thread, read status from CH340 port and publish.
    """
    global stat_ser, state_publisher, running
    rospy.loginfo("Status reader thread started.")

    # Buffer
    line_buffer = ""

    while running and not rospy.is_shutdown():
        if stat_ser is None:
            time.sleep(0.5)
            continue
        
        try:
            # Read a line of data (note: Arduino println ends with '\n')
            # readline() will block until timeout or '\n' is read
            line = stat_ser.readline()

            if not line:
                # Timeout, no data, continue loop
                continue

            line = line.strip() # .strip() works on bytes

            # Simple packet validation
            # [Modified] Changed str '...' to bytes b'...' to match 'line' type
            if not line.startswith(b'<') or not line.endswith(b'>'):
                if line: # Print non-empty invalid data
                    rospy.logwarn("Received malformed status: %s", line)
                continue

            # Parse data: <id1, pos_n1, tgt_n1, pwm_n1, id2, pos_n2, tgt_n2, pwm_n2>
            # [Modified] split() also needs bytes parameter
            parts = line[1:-1].split(b',')
            if len(parts) != 8:
                rospy.logwarn("Received incomplete status packet: %s", line)
                continue

            # Convert all parts to float
            data = [float(p) for p in parts]

            # Create Joy message
            state_msg = Joy()
            state_msg.header.stamp = rospy.Time.now()
            state_msg.header.frame_id = "gripper_base"

            # buttons store integer IDs
            # data[0] = id1, data[4] = id2
            state_msg.buttons = [int(data[0]), int(data[4])]

            # axes store normalized floats
            # [pos_n1, tgt_n1, pwm_n1, pos_n2, tgt_n2, pwm_n2]
            state_msg.axes = [data[1], data[2], data[3], data[5], data[6], data[7]]

            # Publish message
            if state_publisher:
                state_publisher.publish(state_msg)
            
        except serial.SerialException as e:
            rospy.logerr("Serial error in status thread: %s", e)
            if running:
                rospy.logerr("Status port error. Thread stopping.")
                running = False # Stop all loops
            break
        except IOError as e:
             if not running: # Ignore IO errors during normal shutdown
                break
             rospy.logerr("IO error in status thread: %s", e)
             break
        except ValueError as e:
            rospy.logwarn("Value error parsing status (likely bad data): %s", e)
        except Exception as e:
            rospy.logerr("Unhandled error in status thread: %s", e)
            
    rospy.loginfo("Status reader thread stopping.")

# --- 2. Command subscriber callback ---

def command_callback(msg):
    """
    Handle incoming JSON strings from /gripper/command topic.
    """
    global cmd_ser, running, lock

    if cmd_ser is None or not cmd_ser.is_open or not running:
        rospy.logwarn("Command callback ignored: Serial port not ready or node shutting down.")
        return

    rospy.loginfo("Received command msg: %s", msg.data)

    # 1. Parse JSON
    try:
        cmd_dict = json.loads(msg.data)
    except json.JSONDecodeError:
        rospy.logerr("Failed to decode JSON command: %s", msg.data)
        return

    # 2. Validate command
    if cmd_dict.get('node') != 'gripper':
        rospy.logwarn("Command ignored (node != 'gripper'): %s", msg.data)
        return

    command = cmd_dict.get('command')
    value = cmd_dict.get('value')

    arduino_cmd = None

    # 3. Convert command format
    try:
        if command == 'init':
            arduino_cmd = '<"initialization">'
        elif command == 'open':
            arduino_cmd = '<"open">'
        elif command == 'pos' and isinstance(value, list) and len(value) == 2:
            # Ensure values are floats and format
            p1 = float(value[0])
            p2 = float(value[1])
            arduino_cmd = '<"pos", {:.4f}, {:.4f}>'.format(p1, p2)
        elif command == 'pwm' and isinstance(value, list) and len(value) == 2:
            # Ensure values are floats and format
            v1 = float(value[0])
            v2 = float(value[1])
            arduino_cmd = '<"pwm", {:.4f}, {:.4f}>'.format(v1, v2)
        else:
            rospy.logerr("Unknown or malformed command type: %s", msg.data)
            return
    except (TypeError, ValueError) as e:
        rospy.logerr("Error converting command values: %s. Data: %s", e, msg.data)
        return

    # 4. Send command
    if arduino_cmd:
        try:
            with lock: # Use lock for atomic write operations
                rospy.loginfo("Sending to Arduino: %s", arduino_cmd)
                # Must add '\n' (newline) because Arduino uses readStringUntil('\n') or readline()
                cmd_ser.write((arduino_cmd + '\n').encode('utf-8'))
        except serial.SerialException as e:
            rospy.logerr("Failed to send command: %s", e)
        except Exception as e:
            rospy.logerr("Error writing to serial port: %s", e)
            
# --- 3. Main function and shutdown hooks ---

def cleanup():
    """
    ROS shutdown hook for cleaning up resources.
    """
    global running, cmd_ser, stat_ser
    rospy.loginfo("Shutting down gripper node...")

    running = False # Signal all threads to stop

    # Wait for threads to notice the flag
    time.sleep(0.2) 
    
    try:
        if cmd_ser and cmd_ser.is_open:
            rospy.loginfo("Closing command port: %s", cmd_port_name)
            cmd_ser.close()
    except Exception as e:
        rospy.logerr("Error closing command port: %s", e)
        
    try:
        if stat_ser and stat_ser.is_open:
            rospy.loginfo("Closing status port: %s", stat_port_name)
            stat_ser.close()
    except Exception as e:
        rospy.logerr("Error closing status port: %s", e)
        
    rospy.loginfo("Gripper node shutdown complete.")

def gripper_node():
    """
    Main function: Initialize node, ports, and threads.
    """
    global cmd_port_name, stat_port_name, cmd_ser, stat_ser, state_publisher, running

    rospy.init_node('gripper_controller', anonymous=True)
    rospy.on_shutdown(cleanup) # Register shutdown hook

    rospy.loginfo("Starting gripper node...")

    # --- 1. Find ports ---
    try:
        cmd_port_name = force_gripper.utils.find_port_by_name("gripper")
        stat_port_name = force_gripper.utils.find_port_by_name("gripper_ch340")

        if not cmd_port_name or not stat_port_name:
            rospy.logfatal("Could not find required serial ports. Exiting.")
            rospy.logfatal("Gripper (command) port: %s", cmd_port_name)
            rospy.logfatal("CH340 (status) port: %s", stat_port_name)
            return

        rospy.loginfo("Found command port: %s", cmd_port_name)
        rospy.loginfo("Found status port: %s", stat_port_name)

    except Exception as e:
        rospy.logfatal("Error finding ports: %s", e)
        return

    # --- 2. Open serial ports ---
    # Must match baud rate on Arduino
    CMD_BAUD = 1000000
    STAT_BAUD = 1000000

    try:
        # timeout=1 (seconds)
        cmd_ser = serial.Serial(cmd_port_name, CMD_BAUD, timeout=1)
        rospy.loginfo("Command port opened.")
    except serial.SerialException as e:
        rospy.logfatal("Failed to open command port %s: %s", cmd_port_name, e)
        return

    try:
        # timeout=1 (seconds), for readline()
        stat_ser = serial.Serial(stat_port_name, STAT_BAUD, timeout=1)
        rospy.loginfo("Status port opened.")
    except serial.SerialException as e:
        rospy.logfatal("Failed to open status port %s: %s", stat_port_name, e)
        if cmd_ser:
            cmd_ser.close() # Clean up opened ports
        return

    # --- 3. Setup ROS publishers/subscribers ---
    state_publisher = rospy.Publisher('/gripper/state', Joy, queue_size=10)
    rospy.Subscriber('/gripper/command', String, command_callback, queue_size=10)
    rospy.loginfo("ROS topics initialized.")

    # --- 4. Start status reader thread ---
    t = threading.Thread(target=status_reader_thread)
    t.daemon = True # Set as daemon thread, allows Ctrl+C to exit
    t.start()

    # --- 5. Loop and wait ---
    rospy.loginfo("Gripper node running. Waiting for commands...")
    rospy.spin() # Main thread waits for callbacks and shutdown signals

if __name__ == '__main__':
    try:
        gripper_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Ensure cleanup is called even if initialization fails
        if running:
            cleanup()