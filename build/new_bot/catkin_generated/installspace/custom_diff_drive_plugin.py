#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import math


class DifferentialDrivePlugin(Node)
    def __init__(self):
        super().__init__('differentialdriveplugin')
        
        # Parameters for encoder and robot properties
        self.q = 360           # Ticks per revolution, matching the encoder
        self.c = 0.3           # Wheel circumference in meters
        self.R_w = 0.15        # Half the distance between the wheels

        # Initial wheel encoder tick counts
        self.left_ticks_prev = 0
        self.right_ticks_prev = 0
        self.left_ticks = 0
        self.right_ticks = 0

        # Robot's position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        #time
        self.last_time = self.get_clock().now()


        # Publisher and Subscriber
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Int32MultiArray, '/wheel_encoder_ticks', self.encoder_callback, 10)

        # Set a timer to update odometry at 10 Hz
        self.timer = self.create_timer(0.1, self.update_odometry)

        #Timer to update velocity at 10Hz
        self.timer = self.create_timer(0.1, self.update_velocity)

    def encoder_callback(self, msg):
        # Retrieve current ticks from the published message
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]

    def update_odometry(self):
        # Calculate distance traveled by each wheel
        self.dl = ((self.left_ticks - self.left_ticks_prev) / self.q) * self.c
        self.dr = ((self.right_ticks - self.right_ticks_prev) / self.q) * self.c

        # Store current ticks as previous ticks for next iteration
        self.left_ticks_prev = self.left_ticks
        self.right_ticks_prev = self.right_ticks

        # Calculate change in orientation and robot distance
        delta_theta = (self.dr - self.dl) / (2 * self.R_w)
        d = (self.dl + self.dr) / 2

        # Update robot's position and orientation
        self.x += d * math.cos(self.theta + delta_theta / 2)
        self.y += d * math.sin(self.theta + delta_theta / 2)
        self.theta += delta_theta

        # Publish the updated odometry information
        self.publish_odometry()

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # Set the orientation as a quaternion
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

    def update_velocity(self):
        current_time = self.get_clock().now()
        dt = dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = current_time

        # Calculate wheel velocities
        v_l = self.dl / dt
        v_r = self.dr / dt

         # Calculate the robot's linear and angular velocities
        linear_velocity = (v_l + v_r) / 2
        angular_velocity = (self.R_w / 2) * (v_r - v_l)

        twist_msg = Twist()     
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    plugin_node = DifferentialDrivePlugin()
    rclpy.spin(plugin_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
