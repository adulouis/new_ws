#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty

linear_speed = 0.5
angular_speed = 1.0
max_linear_speed = 2.0
max_angular_speed = 2.0

move_bindings = {
    'w': (1, 0),  # Move forward
    's': (-1, 0), # Move backward
    'a': (0, 1),  # Turn left
    'd': (0, -1)  # Turn right
}

speed_bindings = {
    'q': (1.1, 1.1),   # Increase both linear and angular speed
    'z': (0.9, 0.9),   # Decrease both linear and angular speed
    'e': (1.1, 1.0),   # Increase linear speed
    'c': (0.9, 1.0),   # Decrease linear speed
    'r': (1.0, 1.1),   # Increase angular speed
    'v': (1.0, 0.9)    # Decrease angular speed
}


def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear, target_angular):
    return f"currently:\tlinear {target_linear:.2f}\tangular {target_angular:.2f}"


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

     # Initialize ROS node
    rospy.init_node('custom_controller_node')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Initialize velocities
    target_linear = 0.0
    target_angular = 0.0
    linear_scale = linear_speed
    angular_scale = angular_speed


    try:
        print("Use WASD keys to control the robot, 'q'/'z' to adjust speeds, and CTRL+C to quit.")
        print(vels(linear_scale, angular_scale))

        while not rospy.is_shutdown():
            key = get_key()


# Adjust velocities based on key presses
            if key in move_bindings:
                linear_direction, angular_direction = move_bindings[key]
                target_linear = linear_direction * linear_scale
                target_angular = angular_direction * angular_scale
            elif key in speed_bindings:
                linear_scale *= speed_bindings[key][0]
                angular_scale *= speed_bindings[key][1]

                # Ensure speeds do not exceed limits
                linear_scale = min(max_linear_speed, max(0, linear_scale))
                angular_scale = min(max_angular_speed, max(0, angular_scale))

                print(vels(linear_scale, angular_scale))
        
            else:
                # Stop when no valid key is pressed
                target_linear = 0.0
                target_angular = 0.0
                if key == '\x03':  # Ctrl-C to quit
                    break
            
            # Publish velocities to /cmd_vel
            twist = Twist()
            twist.linear.x = target_linear
            twist.angular.z = target_angular
            pub.publish(twist)  

    except Exception as e:
        print(e)

    finally:
        # Stop the robot when the program exits
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



