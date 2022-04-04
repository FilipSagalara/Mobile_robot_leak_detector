#! /usr/bin/env python3
#! /usr/bin/env bash
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import math
from geometry_msgs.msg import Twist, TransformStamped, Vector3, Transform


class Spinner:
    def spin(self):

        rospy.init_node("Spinner")
        cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rate = rospy.Rate(1)
        rospy.sleep(2.0)

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1.0

        cmd_pub.publish(msg)
        rospy.sleep(1.0)

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        cmd_pub.publish(msg)
        rate.sleep()

    def get_rotation(self, msg):
        pass


if __name__ == "__main__":

    for i in range(10):

        spin()
