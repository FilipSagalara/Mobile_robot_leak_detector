#! /usr/bin/env python3
#! /usr/bin/env bash
from p2p_nav.msg import ScanAction, ScanGoal
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction
from visualization_msgs.msg import Marker
import rospy
import roslib

roslib.load_manifest("p2p_nav")

from p2p import P2PNav


class RobotController:
    def __init__(self):
        self.point_to_point_navigation = P2PNav()


if __name__ == "__main__":

    r = RobotController()

    if r.point_to_point_navigation.get_goals() == 2:
        r.point_to_point_navigation.run()

    pass
