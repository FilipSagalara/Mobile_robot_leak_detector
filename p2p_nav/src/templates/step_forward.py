#!/usr/bin/env python3
#!/usr/bin/env bash
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math
from std_srvs.srv import Trigger, TriggerResponse


class StepForward:
    def __init__(self):
        rospy.init_node("Step_forward")
        self.roll = self.pitch = self.yaw = 0.0
        self.target_angle = 30
        self.kP = 1.5
        self.done = False

        self.sub = rospy.Subscriber("/odom", Odometry, self.get_rotation)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.command = Twist()
        self.rate = rospy.Rate(10)

        self.my_service = rospy.Service("/step_forward", Trigger, self.trigger_response)

    def odom_callback(self, msg):
        pass


if __name__ == "__main__":

    s = Spin()
    print(s.yaw)

    rospy.spin()
