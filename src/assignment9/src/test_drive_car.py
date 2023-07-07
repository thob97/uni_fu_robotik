#!/usr/bin/env python
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
import time

#this file is used for test driving only

def drive(secs, angle, speed):
    
    pub_1 = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    pub_2 = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)

    #speed and angle
    steeringCommand = NormalizedSteeringCommand(header= None, value = angle)
    speedCommand = SpeedCommand(header= None, value = speed)

    #drive
    print('dirve now')
    start = time.time()
    while not rospy.is_shutdown() and time.time() - start < secs:
        pub_1.publish(steeringCommand)
        pub_2.publish(speedCommand)
        rospy.sleep(0.1)

    #stop
    speedCommand = SpeedCommand(header= None, value = 0.0)
    pub_2.publish(speedCommand)

if __name__ == '__main__':
    try:
        rospy.init_node('publisher', anonymous=True)
        #2 sekunden
        #drive(2, 0, 0.3)

        #2 meter
        #drive(6.67, 1.0, 0.3)
        #drive(6.67, 0, 0.3)
        #drive(6.67, -1.0, 0.3)

        #as fast as possible for 10 secs
        drive(10, 0, 1)

        #left
        #drive(1, 1, 1)
        #right
        #drive(1, -1, 1)

    except rospy.ROSInterruptException:
        pass