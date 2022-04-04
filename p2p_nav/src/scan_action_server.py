#! /usr/bin/env python2.7
#! /usr/bin/env bash
import roslib

roslib.load_manifest("p2p_nav")
import rospy
from geometry_msgs.msg import PointStamped, Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from p2p_nav.msg import ScanAction, ScanGoal, ScanFeedback, ScanResult
from std_msgs.msg import Int16


# ROS Server for performing 360 scan


class ScanServer(object):

    _feedback = ScanFeedback()
    _result = ScanResult()
    _done = False

    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "scanner", ScanAction, execute_cb=self.execute
        )
        self.server.start()
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.audio_subscriber = rospy.Subscriber("dB", Int16)

        self.rotate = Twist()
        self.rotate.linear.x = 0
        self.rotate.linear.y = 0
        self.rotate.linear.z = 0
        self.rotate.angular.x = 0
        self.rotate.angular.y = 0
        self.rotate.angular.z = 0.5  # rotate right, 1 for rotate level
        self.dB_lvl = 0

    def dB_callback(self, msg):
        print("callback")
        pass

    def execute(self, goal):
        # robot_moves_here
        rospy.loginfo("Performing 360 scan")
        self.cmd_vel_pub.publish(self.rotate)
        rospy.sleep(2)
        self.server.set_succeeded()

    def wait(self):
        while not self._done:
            print("Wating for rotation to complete")
            rospy.Rate(1)
        self._done = True
