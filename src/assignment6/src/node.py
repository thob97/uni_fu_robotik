#!/usr/bin/env python
import rospy
import cv2 as cv
from autominy_msgs.msg import Tick
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from collections import deque
import message_filters
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
import time
from sensor_msgs.msg import PointCloud2, LaserScan

#Quelle für ROS Klassen Struktur
#https://stackoverflow.com/questions/37373211/update-the-global-variable-in-rospy

#rostopic echo "/autominy_msgs/Speed"
#rostopic echo "/sensors/arduino/ticks"
#rqt_plot "/autominy_msgs/Speed"

class MyNode():
    def __init__(self):
        self.none
        #paras

        #pubs

        #subs
        rospy.Subscriber('/sensors/rplidar/scan', LaserScan, self.task6_a)

    #calculates total_ticks/ distance ratio
    def task6_a(self, angle):
        
        #drive 2m at 0.3m/s
        meter = 2
        speed = 0.3
        self.drive_until((meter/speed),angle,speed)

        print(f'speed:{speed}, steering:{angle}, tick count:{self.tick_counter}, distance:{meter} ,tick / distance ratio:{self.tick_counter/meter}')
        return (speed, angle, self.tick_counter/meter)

    def drive_until(self, secs, angle, speed):
        pub_1 = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
        pub_2 = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)

        #speed and angle
        steeringCommand = NormalizedSteeringCommand(header= None, value = angle)
        speedCommand = SpeedCommand(header= None, value = speed)

        #drive
        start = time.time()
        while not rospy.is_shutdown() and time.time() - start < secs:
            pub_1.publish(steeringCommand)
            pub_2.publish(speedCommand)
            rospy.sleep(0.1)

        #stop
        speedCommand = SpeedCommand(header= None, value = 0.0)
        pub_2.publish(speedCommand)

    def count_ticks(self, tick:Tick):
        self.tick_counter += 1



    #calculates and publishes velocity
    #use... to plot it
    #rqt_plot "/autominy_msgs/Speed"
    def task5_3b(self):
        sub1 = message_filters.Subscriber('/simulation/odom_ground_truth', Odometry)
        sub2 = message_filters.Subscriber('/sensors/arduino/ticks', Tick)
        ts = message_filters.TimeSynchronizer([sub1, sub2], 10)
        ts.registerCallback(self.callback)
        self.publish_median_ticks()

    def callback(self, obj:Odometry, tick:Tick):
        #calc dist
        self.update_distance(obj)
        #calc verlocity
        self.verlocity_queue_sum -= self.verlocity_queue.popleft()
        self.verlocity_queue_sum += self.dist_meter
        self.verlocity_queue.append(self.dist_meter)

    def update_distance(self, obj:Odometry):
        if self.x == None and self.y == None:
            self.x = abs(obj.pose.pose.position.x)
            self.y = abs(obj.pose.pose.position.y)
        else:
            new_x = abs(obj.pose.pose.position.x)
            new_y = abs(obj.pose.pose.position.y)
            self.dist_meter = abs(self.x - new_x) + abs(self.y - new_y)
            self.x = new_x
            self.y = new_y

    def publish_median_ticks(self):
        pub = rospy.Publisher('/autominy_msgs/Speed', Float64, queue_size=10)
        while not rospy.is_shutdown():
            median = self.verlocity_queue_sum / self.verlocity_queue.maxlen
            pub.publish(median)
            rospy.sleep(0.1)



def main():
    rospy.init_node('Ticks_to_Speed_Publisher', anonymous=True)
    node = MyNode()
    #node.task5_3a(-1)
    #node.task5_3b()

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

