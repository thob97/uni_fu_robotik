#!/usr/bin/env python
import rospy
import time
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
from sensor_msgs.msg import LaserScan



class MyNode():
    def __init__(self):
        self.laser_ranges:LaserScan = [float('Inf') for _ in range(360)]
        #paras

        #pubs
        self.pub_steering = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
        self.pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)

        #subs
        rospy.Subscriber('/sensors/rplidar/scan', LaserScan, self.getLaserRanges)

    def getLaserRanges(self, msg:LaserScan):
        self.laser_ranges=msg.ranges

    def task_1(self, min_dist, min_camera_front_angle, min_camera_left_right_angle):
        while not rospy.is_shutdown():
            #if wall is close to front of car 
            front = self.laser_ranges[-min_camera_front_angle//2:] + self.laser_ranges[:min_camera_front_angle//2]
            if min(front) < min_dist:
                left = self.laser_ranges[min_camera_front_angle:min_camera_left_right_angle]
                right = self.laser_ranges[-min_camera_left_right_angle:-min_camera_front_angle]
                
                #turn left
                if min(left) > min(right):
                    steeringCommand = NormalizedSteeringCommand(header= None, value = 1)
                    speedCommand = SpeedCommand(header= None, value = 0.3)

                #turn right
                else:
                    steeringCommand = NormalizedSteeringCommand(header= None, value = -1)
                    speedCommand = SpeedCommand(header= None, value = 0.3)

            #drive straight
            else:
                steeringCommand = NormalizedSteeringCommand(header= None, value = 0)
                speedCommand = SpeedCommand(header= None, value = 0.3)

            #publish
            self.pub_steering.publish(steeringCommand)
            self.pub_speed.publish(speedCommand)
            rospy.sleep(0.1)


    #unused
    def publish_dummy(self):
        pub = rospy.Publisher('/autominy_msgs/Speed', LaserScan, queue_size=10)
        while not rospy.is_shutdown():
            data = 0
            pub.publish(data)
            rospy.sleep(0.1)



def main():
    rospy.init_node('Laser_Scan_Publisher', anonymous=True)
    node = MyNode()
    node.task_1(1, 15, 30)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
        cv.DestroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

