#!/usr/bin/env python2.7
#!/usr/bin/env bash

import pyaudio
import rospy
from leak_detector import LeakDetector
from std_msgs.msg import Float64

def main():
    rospy.init_node("audio_node")
    audio_pub = rospy.Publisher("/dB", Float64, queue_size=1)
    detector = LeakDetector()
    rate = rospy.Rate(10)  # Hz
    msg = Float64()

    try:
        while not rospy.is_shutdown():
            detector.listen()
            msg.data = detector.get_db()
            audio_pub.publish(msg)
            rate.sleep()
    except KeyboardInterrupt:
        print("Program exit by user")

if __name__ == "__main__":
    main()
