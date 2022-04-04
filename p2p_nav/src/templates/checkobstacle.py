#!/usr/bin/env python3
#!/usr/bin/env bash
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class CheckObstacle:
    def __init__(self):
        rospy.init_node("measure_distance")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(5)

        self.is_obstacle = False

    def callback(self, msg):

        print(msg.range)
        if msg.range < 0.4:

            print("Obstacle detected")
            self.is_obstacle = True
        else:
            self.is_obstacle = False
            pass

    def run(self):

        while not rospy.is_shutdown():
            self.sub = rospy.Subscriber("/range/fr", Range, self.callback)
            self.rate.sleep()

    def get_status(self):
        return self.is_obstacle


if __name__ == "__main__":

    c = CheckObstacle()
    c.run()
