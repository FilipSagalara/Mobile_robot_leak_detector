#!/usr/bin/env python2.7
#!/usr/bin/env bash

import rospy
from leakpoint import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse, Empty

class RvizManager:
    _CONST_NUMSPACE = 1  # With _CONST_NUMSPACE only one marker is created

    def __init__(self):
        rospy.loginfo("Rviz manager for leak application initialized")
        print("Rviz manager initialized")
        rospy.init_node("rviz_leaks_manager")

        # Subscribers
        self.clicked_point_subscriber = rospy.Subscriber(
            "/clicked_point", PointStamped, self.send_point_on_click
        )
        self.robot_pose_sub = rospy.Subscriber(
            "/Pose", PoseStamped, self.create_point_on_robot_position
        )
        self.dB_sub = rospy.Subscriber("/dB", Float64, self.send_leak_estimate)

        # Services
        self.send_leak_point_srv = rospy.Service("/tag_leak", Trigger, self.leak_tag)
        self.make_snapshot_srv = rospy.Service("/image_saver/save", Empty, self.take_picture)

        # Initialize robot pose
        self.robot_pose_x = 0
        self.robot_pose_y = 0
        self.robot_pose_z = 0

        # Initialize state variables
        self.counter = 0
        self.is_tagged = False

        # Colors
        self._red = (1, 0, 0)
        self._green = (0, 1, 0)
        self._yellow = (1, 1, 0)

    # Callbacks
    def create_point_on_robot_position(self, msg):
        point = Point("robot_following_points")
        point.set_numspace(self.counter)

        self.robot_pose_x = msg.pose.position.x
        self.robot_pose_y = msg.pose.position.y
        self.robot_pose_z = 0

        point.set_marker_position_relative_to_robot(
            self.robot_pose_x, self.robot_pose_y, self.robot_pose_z
        )

    def send_point_on_click(self, msg):
        point_clicked = Point("clicked_points_ns")
        point_clicked.set_numspace(self._CONST_NUMSPACE)
        self.counter += 1

        pose_x = msg.point.x
        pose_y = msg.point.y
        pose_z = msg.point.z

        point_clicked.set_marker_position_on_map(pose_x, pose_y, pose_z)

    def send_leak_estimate(self, msg):
        self.is_tagged = False
        point = Point("/visualization_marker")
        offset = 2.0 - (msg.data * (1.0 / 60))  # Normalize

        if msg.data < 20:
            point.set_color(self._green)
        elif 30 < msg.data < 50:
            point.set_color(self._yellow)
        elif msg.data > 60:
            point.set_color(self._red)

        point.set_marker_position_relative_to_robot(
            self.robot_pose_x + offset, self.robot_pose_y, self.robot_pose_z
        )

    def leak_tag(self, req):
        point = Point("/leak_tag")
        point.set_type("cylinder")
        point.set_marker_position_relative_to_robot(
            self.robot_pose_x, self.robot_pose_y, self.robot_pose_z
        )

        return TriggerResponse(
            success=self.is_tagged, message="Hey, roger that; we'll be right there!"
        )

    def take_picture(self, req):
        print("Taking picture")
        pass

if __name__ == "__main__":
    rviz_manager = RvizManager()
    rospy.spin()
