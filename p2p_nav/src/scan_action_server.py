#!/usr/bin/env python2.7

import roslib
roslib.load_manifest("p2p_nav")
import rospy
from geometry_msgs.msg import Twist
import actionlib
from p2p_nav.msg import ScanAction, ScanFeedback, ScanResult
from std_msgs.msg import Int16


# ROS Server for performing 360 scan
class ScanServer(object):

    _feedback = ScanFeedback()
    _result = ScanResult()
    _done = False

    def __init__(self):
        # Initialize the action server
        self.server = actionlib.SimpleActionServer("scanner", ScanAction, execute_cb=self.execute)
        self.server.start()

        # Publisher for robot velocity
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Subscriber for audio level
        self.audio_subscriber = rospy.Subscriber("dB", Int16, self.dB_callback)

        # Initialize the rotation command
        self.rotate = Twist()
        self.rotate.linear.x = 0
        self.rotate.linear.y = 0
        self.rotate.linear.z = 0
        self.rotate.angular.x = 0
        self.rotate.angular.y = 0
        self.rotate.angular.z = 0.5  # rotate right, 1 for rotate level

        self.dB_lvl = 0

    def dB_callback(self, msg):
        rospy.loginfo("Audio level callback triggered")
        self.dB_lvl = msg.data

    def execute(self, goal):
        # Perform a 360 scan
        rospy.loginfo("Performing 360 scan")
        self.cmd_vel_pub.publish(self.rotate)
        rospy.sleep(2)
        self.server.set_succeeded()

    def wait(self):
        # Wait for rotation to complete
        while not self._done:
            rospy.loginfo("Waiting for rotation to complete")
            rospy.sleep(1)
        self._done = True


if __name__ == "__main__":
    try:
        rospy.init_node("scan_server")
        server = ScanServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
