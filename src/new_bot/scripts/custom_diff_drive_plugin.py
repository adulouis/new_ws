import math
import rospy
from geometry_msgs.msg import Twist
import tf
from tf.transformations import quaternion_from_euler



class CustomDrivePlugin:
    def __init__(self):
        rospy.init_node('diff_drive_plugin', anonymous=True)

        #Parameters for robot physical properties
        self.wheel_base = rospy.get_param("wheel_base")  # Distance between wheels
        self.wheel_radius = rospy.get_param("wheel_radius")  # Wheel radius

        # Robot's position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    
        rospy.Subscriber("/wheel_encoder_ticks", type, encoder_callback)


    
    def encoder_callback(msg):
    # Process the encoder tick data here
    encoder_ticks = msg.data
    rospy.loginfo("Received encoder ticks: %d", encoder_ticks)
    
        

