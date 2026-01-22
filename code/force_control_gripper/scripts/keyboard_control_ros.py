#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS 1 keyboard control node.

Reads individual key presses and publishes them as JSON commands to /gripper/command topic.
Must be run simultaneously with gripper_ros_node.py.
"""

import rospy
from std_msgs.msg import String
import json
import sys
import tty
import termios

# --- Global variables ---
pub = None

# --- Key mappings ---
# You can modify the PWM speed here
PWM_SPEED = 0.3 # (between 0.0 and 1.0)

key_bindings = {
    # Commands
    'i': ('init', None),         # Initialize
    'o': ('open', None),         # Open (position 0.0)

    # Position control
    'c': ('pos', [1.0, 1.0]),    # "Close" (close to position 1.0)
    'p': ('pos', [0.5, 0.5]),    # "Position" (to middle position 0.5)

    # PWM control
    'w': ('pwm', [PWM_SPEED, PWM_SPEED]),     # "Forward" (close both together)
    's': ('pwm', [-PWM_SPEED, -PWM_SPEED]),   # "Back" (open both together)
    'a': ('pwm', [PWM_SPEED, -PWM_SPEED]),    # "Left" (left close, right open)
    'd': ('pwm', [-PWM_SPEED, PWM_SPEED]),    # "Right" (left open, right close)
    ' ': ('pwm', [0.0, 0.0]),                 # "Stop" (PWM set to 0)

    # Exit
    'q': ('quit', None)
}

def print_instructions():
    """Print help menu"""
    print("\n" + "="*20)
    print(" Gripper Keyboard Controller")
    print("="*20)
    print(" i : Initialize (Initialization)")
    print(" o : Fully open (Open)")
    print(" c : Fully close (Close - Position)")
    print(" p : Move to mid-position (Mid-Position)")
    print(" ----- PWM Control -----")
    print(" w : Close both together")
    print(" s : Open both together")
    print(" a : Left close, right open")
    print(" d : Left open, right close")
    print(" [space] : PWM stop (PWM=0)")
    print("\n q : Quit")
    print("="*20)
    print("Press a key:")

def send_command(command, value=None):
    """
    Package command into JSON and publish via ROS.
    """
    global pub
    if rospy.is_shutdown() or pub is None:
        return

    # 1. Build command dictionary
    command_dict = {'node': 'gripper', 'command': command}
    if value is not None:
        command_dict['value'] = value

    # 2. Convert to JSON string
    command_json = json.dumps(command_dict)

    # 3. Publish
    rospy.loginfo("Sending: %s", command_json)
    pub.publish(command_json)

def main_loop():
    """
    Main loop: Capture key presses and send commands.
    """
    global pub

    # --- ROS initialization ---
    rospy.init_node('gripper_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/gripper/command', String, queue_size=10)
    rospy.loginfo("Keyboard controller started, publishing to /gripper/command")

    # --- Terminal setup ---
    # Save current terminal settings
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    # Register ROS shutdown hook to restore terminal settings
    def restore_terminal_settings():
        rospy.loginfo("Restoring terminal settings...")
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    rospy.on_shutdown(restore_terminal_settings)

    try:
        # --- Set terminal to "cbreak" mode ---
        # This allows reading individual keys without pressing Enter
        tty.setcbreak(sys.stdin.fileno())

        print_instructions()

        while not rospy.is_shutdown():
            # 1. Read single character
            key = sys.stdin.read(1)

            # 2. Look up key binding
            if key in key_bindings:
                command, value = key_bindings[key]

                if command == 'quit':
                    rospy.loginfo("Received quit key 'q', shutting down...")
                    break # Exit while loop

                # 3. Send command
                send_command(command, value)

            else:
                rospy.logwarn("Unknown key: %s", key)
                print_instructions() # Re-print menu

    except rospy.ROSInterruptException:
        # If ROS shuts down (e.g., Ctrl+C), do nothing
        pass
    except Exception as e:
        rospy.logerr("Error occurred: %s", e)
    finally:
        # --- Restore terminal ---
        # Ensure terminal is restored regardless
        restore_terminal_settings()
        rospy.loginfo("Keyboard controller closed.")

if __name__ == '__main__':
    try:
        main_loop()
    except Exception as e:
        print("Startup error: %s", e)