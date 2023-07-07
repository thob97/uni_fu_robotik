#!/usr/bin/env python
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand

def talk():
    rospy.init_node('publisher', anonymous=True)
    pub_1 = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    pub_2 = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)

    steeringCommand = NormalizedSteeringCommand(header= None, value = 1.0)
    speedCommand = SpeedCommand(header= None, value = 0.3)

    while not rospy.is_shutdown():
        pub_1.publish(steeringCommand)
        pub_2.publish(speedCommand)
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        talk()
    except rospy.ROSInterruptException:
        pass