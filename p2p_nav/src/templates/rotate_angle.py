#!/usr/bin/env python3
#!/usr/bin/env bash
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math
from std_srvs.srv import Trigger, TriggerResponse


class Spin:
    def __init__(self):
        rospy.init_node("my_quaternion_to_euler")
        self.roll = self.pitch = self.yaw = 0.0
        self.target_angle = 30
        self.kP = 2.5
        self.done = False

        self.sub = rospy.Subscriber("/odom", Odometry, self.get_rotation)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.command = Twist()
        self.rate = rospy.Rate(10)

        self.my_service = rospy.Service("/rotate", Trigger, self.trigger_response)

    def get_rotation(self, msg):

        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        # print (yaw)

    def spin(self, angle):

        self.done = False
        curr_yaw = self.yaw
        while not self.done:

            target_rad = angle * math.pi / 180 + curr_yaw
            self.command.angular.z = self.kP * (target_rad - self.yaw)
            print("target_rad {}, self yaw {}".format(target_rad, self.yaw))
            rospy.sleep(1)
            self.pub.publish(self.command)

            if self.command.angular.z < 0.1:
                print(self.command.angular.z)
                print("move_done")
                self.done = True
                self.robot_stop()

    def robot_stop(self):
        if self.done == True:

            self.command.linear.x = 0
            self.command.linear.y = 0
            self.command.linear.z = 0

            self.command.angular.x = 0
            self.command.angular.y = 0
            self.command.angular.z = 0
            self.pub.publish(self.command)
        else:
            pass

    def get_status(self):
        return self.done

    def trigger_response(self, request):
        self.spin(30)

        return TriggerResponse(
            success=True, message="Hey, roger that; we'll be right there!"
        )

    def wait_for_spin(self):
        pass


if __name__ == "__main__":

    s = Spin()
    print(s.yaw)

    rospy.spin()
