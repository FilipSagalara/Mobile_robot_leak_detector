#!/usr/bin/env python2.7

import rospy
import roslib
import actionlib
from p2p_nav.msg import ScanAction, ScanGoal
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

roslib.load_manifest("p2p_nav")


class P2PNav:
    def __init__(self):
        rospy.loginfo("P2PNav is initializing")
        self.points = []

        rospy.init_node("Rviz_marker_manager")
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
        rospy.Subscriber("/clicked_point", PointStamped, self.clicked_callback)
        self.counter = 0

        rospy.loginfo("Move base client initialized....")
        rospy.loginfo("360scanner client initialized....")

    def clicked_callback(self, msg):
        rospy.loginfo("Callback activated")
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = msg.header.stamp

        # Set shape, Arrow: 0; Cube: 1; Sphere: 2; Cylinder: 3
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.id = self.counter
        self.counter += 1

        # Set the scale of the marker
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = msg.point.x
        marker.pose.position.y = msg.point.y
        marker.pose.position.z = msg.point.z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        self.marker_pub.publish(marker)
        self.points.append(marker)
        rospy.loginfo(f"Number of points: {len(self.points)}")

    def run(self):
        rospy.loginfo("P2P NAV is running")
        while self.points:
            rospy.loginfo("Moving to point")
            current_goal = self.points.pop(0)
            self.movebase_client(current_goal)

    def movebase_client(self, point):
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point.pose.position.x
        goal.target_pose.pose.position.y = point.pose.position.y
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()

    def get_goals(self):
        return self.points


class RobotController:
    def __init__(self):
        self.point_to_point_navigation = P2PNav()


if __name__ == "__main__":
    robot_controller = RobotController()

    if len(robot_controller.point_to_point_navigation.get_goals()) == 2:
        robot_controller.point_to_point_navigation.run()
