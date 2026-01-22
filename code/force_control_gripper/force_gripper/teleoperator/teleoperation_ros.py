#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
[Modified]
ROS 1 node: Teleoperator to gripper controller.

This node:
1. Reads load values from "teleoperator" serial port (e.g., "L3:50,L4:-20").
2. Normalizes these values (divides by 885.0).
3. Uses normalized values as PWM commands, packaged into JSON strings.
4. Publishes JSON commands to /gripper/command topic.
"""

import rospy
from std_msgs.msg import String # [Modified] Import String to publish JSON
import serial
import time
import force_gripper.utils
import json # [New] Import json

# --- Configuration ---
DEVICE_NAME_KEYWORD = "teleoperator" # Keyword to find the USB device
BAUD_RATE = 115200
MOTOR_IDS = [3, 4]                   # The motor IDs you expect data from
PUBLISH_RATE = 100                   # Desired publish rate in Hz

# Normalization factor (matches Arduino PWM_MAX=885)
NORMALIZATION_FACTOR = 885.0

GRIPPER_OPEN_ACTION = "open" # open or PWM
GRIPPER_CLOSE_ACTION = "average"


def teleop_to_gripper_controller():
    """
    Read "teleoperator" loads and publish them as PWM commands to /gripper/command.
    """

    # [Modified] Publish to /gripper/command topic, type String
    pub = rospy.Publisher('/gripper/command', String, queue_size=1)

    # [Modified] Node name
    rospy.init_node('teleop_to_gripper_controller', anonymous=True)

    # 1. MODIFIED: Find port using the force_gripper utility
    port_name = force_gripper.utils.find_port_by_name(DEVICE_NAME_KEYWORD)
    if not port_name:
        rospy.logerr("Could not find a serial device with the name '%s'. Aborting.", DEVICE_NAME_KEYWORD)
        return

    ser = None
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        rospy.loginfo("Connected to %s. Publishing commands at %dHz...", port_name, PUBLISH_RATE)
    except serial.SerialException as e:
        rospy.logerr("Error opening serial port %s: %s", port_name, e)
        return

    # Dictionary to store the most recent normalized load/pwm value for each motor
    # [Modified] We now use MOTOR_IDS [3, 4] to control gripper [1, 2]
    # We need an ordered mapping, or assume L3 -> gripper1, L4 -> gripper2
    # We assume sorted(MOTOR_IDS) corresponds to [gripper_pwm_1, gripper_pwm_2]
    # e.g., current_loads[3] will be pwm1, current_loads[4] will be pwm2
    current_loads = {mid: 0.0 for mid in MOTOR_IDS}
    
    rate = rospy.Rate(PUBLISH_RATE)

    # 2. MODIFIED: Main loop runs at a fixed rate
    while not rospy.is_shutdown():
        # --- Part A: Read all available serial data ---
        try:
            while ser.in_waiting > 0:
                line = ser.readline()
                if not line:
                    continue
                
                data_str = ""
                try:
                    data_str = line.decode('utf-8').strip() # e.g., "L3:50,L4:-20"
                except UnicodeDecodeError:
                    rospy.logwarn("Unicode decode error. Skipping line: %s", line)
                    continue

                # 3. MODIFIED: Correctly parse the "L<id>:<value>" format
                motor_readings = data_str.split(',') # -> ['L3:50', 'L4:-20']
                for reading in motor_readings:
                    try:
                        id_str, load_str = reading.split(':') # -> 'L3', '50'
                        motor_id = int(id_str.strip('L'))     # -> 3
                        
                        # 1. Convert raw reading (integer) to float
                        raw_load_value = float(load_str)
                        # 2. Normalize (divide by 885)
                        normalized_load = raw_load_value / NORMALIZATION_FACTOR

                        # 3. Store normalized value
                        if motor_id in current_loads:
                            current_loads[motor_id] = normalized_load

                    except (ValueError, IndexError):
                        pass

        except serial.SerialException as e:
            rospy.logerr("Serial communication error: %s", e)
            break
        except Exception as e:
             rospy.logerr("Unhandled error in serial read loop: %s", e)


        # --- Part B: Publish the ROS Command Message at a fixed rate ---

        # [Key modification]
        # 1. Build the PWM values list to send, in MOTOR_IDS order
        # Assuming MOTOR_IDS = [3, 4], we want pwm_values = [current_loads[3], current_loads[4]]
        pwm_values = [current_loads[mid] for mid in sorted(current_loads.keys())]
        pwm_values[1] = -pwm_values[1]

        if(pwm_values[0] < 0.0):
            if(pwm_values[0] > -0.05):
                # pwm_values = [-0.075, -0.075]
                pwm_values = [-0.0, -0.0]
            else:
                pass
            if(GRIPPER_CLOSE_ACTION == "average"):
                pwm_values[0] = sum(pwm_values)/2.0
                pwm_values[1] = pwm_values[0]
            # pwm_values[0] = sum(pwm_values)/2.0
            # pwm_values[1] = pwm_values[0]
            command_dict = {
            'node': 'gripper',
            'command': 'pwm',
            'value': pwm_values # e.g., [0.056, -0.022]
            }
        elif(pwm_values[0] > 0.02):
            if(GRIPPER_OPEN_ACTION == "open"):
                command_dict = {
                'node': 'gripper',
                'command': 'open',
                }
            else:
                if(pwm_values[0] < 0.05):
                    # pwm_values = [-0.075, -0.075]
                    pwm_values = [-0.0, -0.0]
                else:
                    pass
                # pwm_values[0] = sum(pwm_values)/2.0
                # pwm_values[1] = pwm_values[0]
                command_dict = {
                'node': 'gripper',
                'command': 'pwm',
                'value': pwm_values # e.g., [0.056, -0.022]
                }
        else:
            command_dict = {
            'node': 'gripper',
            'command': 'pwm',
            'value': [0, 0] # e.g., [0.056, -0.022]            
            }

        # 2. Build command dictionary


        # 3. Convert to JSON string
        try:
            command_json = json.dumps(command_dict)
        except Exception as e:
            rospy.logerr("Failed to create JSON command: %s", e)
            continue

        # 4. Publish command
        pub.publish(command_json)
        
        rate.sleep() # Wait to maintain the loop rate

    # --- Cleanup ---
    if ser and ser.is_open:
        ser.close()
        rospy.loginfo("Serial port closed.")

if __name__ == '__main__':
    try:
        # [Modified] Call new function name
        teleop_to_gripper_controller()
    except rospy.ROSInterruptException:
        pass