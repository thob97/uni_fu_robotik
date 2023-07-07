#!/usr/bin/env python
import rospy
from autominy_msgs.msg import Speed

def callback(received_msg):
    print("Received msg: ", received_msg)

def listen():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber('/sensors/speed', Speed, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass