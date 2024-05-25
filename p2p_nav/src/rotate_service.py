#!/usr/bin/env python2.7

import rospy
import roslib
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

roslib.load_manifest("p2p_nav")

# Class for rotating robot based on odometry readings
class RotateService(object):
    def __init__(self):
        # Initializing ROS Node
        rospy.init_node("rotation_service")

        # Initialize and advertise the service
        rospy.Service("rotate", Empty, self.callback)

        # Publisher for kinematic task
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Subscriber for reading encoder measurements
        self.odom_sub = rospy.Subscriber("rotate_service_odom", Odometry, self.get_rotating_info)

        # Initialize rotation parameters
        self.rotate_w = 0
        self.rotation = Twist()
        self.rotation.linear.x = 0
        self.rotation.linear.y = 0
        self.rotation.linear.z = 0
        self.rotation.angular.x = 0
        self.rotation.angular.y = 0
        self.rotation.angular.z = 0.5

    def get_rotating_info(self, msg):
        # Update the rotation parameter from odometry readings
        self.rotate_w = msg.pose.pose.orientation.w

    def callback(self, req):
        # Callback for handling the rotation request
        rospy.loginfo("Rotating")
        rate = rospy.Rate(1)
        self.pub.publish(self.rotation)
        rate.sleep()

if __name__ == "__main__":
    try:
        RotateService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
