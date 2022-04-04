#! /usr/bin/env python3
#! /usr/bin/env bash
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import math
from geometry_msgs.msg import Twist, TransformStamped, Vector3, Transform


class Spinner:
    def __init__(self):
        rospy.init_node("Spinner")
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.listener = tf.TransformListener()
        self.done = False
        self.base_cmd = Twist()

    def spin_once(self, rad, clockwise):

        while rad < 0:
            rad += 2 * math.pi

        while rad > 2 * math.pi:
            rad -= 2 * math.pi

        self.listener.waitForTransform(
            "/base_link", "/odom", rospy.Time(0), rospy.Duration(1.0)
        )

        self.start_transform = tf.TransformStamped()
        self.current_transform = tf.TransformStamped()

        self.listener.lookupTransform("/base_link", "/odom", rospy.Time(0))

        self.base_cmd = Twist()
        self.base_cmd.linear.x = self.base_cmd.linear.y = 0.0
        self.base_cmd.angular.z = 0.75
        if clockwise:
            self.base_cmd.angular.z = -1 * self.base_cmd.angular.z

        desired_turn_axis = Vector3(0, 0, 1)
        if not clockwise:
            desired_turn_axis = -desired_turn_axis

        print("Now rotating")
        self.cmd_pub.publish(self.base_cmd)

    def start(self):
        angle = float(1 / 12)  # 30 degree
        self.spin_once(1, 1)


if __name__ == "__main__":

    s = Spinner()
    s.start()
