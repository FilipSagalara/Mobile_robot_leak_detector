#!/usr/bin/env python2.7
#!/usr/bin/env bash

import rospy
from visualization_msgs.msg import Marker
import matplotlib

class Point:
    def __init__(self, namespace):
        self.marker = Marker()
        self.marker_list = []
        self.counter = 0

        # Define publisher and namespace that class publishes to
        self.publisher = rospy.Publisher(namespace, Marker, queue_size=1)

        # Set shape, Arrow: 0; Cube: 1; Sphere: 2; Cylinder: 3
        self.marker.type = 2
        self.marker.action = 0  # Add marker

        # Set the scale of the marker
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5

        # Set the color
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        # Set the pose of the marker
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1

    def set_numspace(self, num):
        self.marker.id = num

    def set_color(self, color):
        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = 1.0

    def set_type(self, shape):
        if shape.lower() == "cube":
            self.marker.type = 1
        elif shape.lower() == "sphere":
            self.marker.type = 2
        elif shape.lower() == "cylinder":
            self.marker.type = 3
        else:
            print("Wrong shape!")

    def set_scale(self, x, y, z):
        self.marker.scale.x = x
        self.marker.scale.y = y
        self.marker.scale.z = z

    def set_marker_position_on_map(self, x, y, z):
        # Position and frame
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.get_rostime()

        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z

        # Orientation
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1

        # Publish on map
        self.publisher.publish(self.marker)
        self.marker_list.append(self.marker)

    def set_marker_position_relative_to_robot(self, x, y, z):
        # Position and frame
        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = rospy.get_rostime()

        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z

        # Orientation
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1

        # Publish relative to robot
        self.publisher.publish(self.marker)
        self.marker_list.append(self.marker)
