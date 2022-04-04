#! /usr/bin/env python2.7
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

#Class for p2p navigation using navigation stack

class P2PNav:
    def __init__(self):
        rospy.loginfo("P2PNav is initializing")
        self.points = []
        
        
        rospy.init_node("Rviz_marker_manager")
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
        rospy.Subscriber("/clicked_point", PointStamped, self.clicked_callback)
        self.counter = 0

        rospy.loginfo("move base client initialized....")
        rospy.loginfo("360scanner client initalized....")

    def clicked_callback(self, msg):
        print("callback dziala")
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = msg.header.stamp

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.action = 0  # add marker

        marker.id = self.counter  # namespace
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
        # print(self.counter)
        self.points.append(marker)
        print(int(len(self.points)))

    def run(self):

        print("P2P NAV")
        rospy.loginfo("P2P_Nav")
        while len(self.points) > 0:
            print("Moving to point")
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

