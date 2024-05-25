#!/usr/bin/env python3

import rospy
import roslib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
import actionlib
from p2p_nav.msg import ScanAction, ScanGoal

roslib.load_manifest("p2p_nav")

from p2p import P2PNav


class RobotController:
    def __init__(self):
        self.point_to_point_navigation = P2PNav()


if __name__ == "__main__":
    rospy.init_node('robot_controller')
    r = RobotController()

    if r.point_to_point_navigation.get_goals() == 2:
        r.point_to_point_navigation.run()
