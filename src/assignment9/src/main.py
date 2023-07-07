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
import threading

#from another file
from test_drive_car import drive


#Quelle für ROS Klassen Struktur
#https://stackoverflow.com/questions/37373211/update-the-global-variable-in-rospy

#rostopic echo "/autominy_msgs/Speed"
#rqt_plot "/autominy_msgs/Speed"

class MyNode():
    def __init__(self):
        #paras
        self.x = None
        self.y = None
        #travled distance
        self.dist_meter = 0
        #puffer for verlocity
        self.verlocity_queue = deque([0 for _ in range(50)],maxlen=50)
        self.verlocity_queue_sum = 0


        #pubs

        #subs





    #calculates and publishes velocity
    #use... to plot it
    #rqt_plot "/autominy_msgs/Speed"
    def task9_b(self):
        rospy.Subscriber('/simulation/odom_ground_truth', Odometry, self.callback)
        #as fast as possible for 10 secs
        #drive(10, 0, 1)
        threading.Thread(group=None, target=drive, args=(10, 0, 1)).start()
        #threading.Thread(target=threadFunc)
        self.publish_median_ticks()

    def callback(self, obj:Odometry):
        #calc dist 
        self.update_distance(obj)
        #calc verlocity
        self.verlocity_queue_sum -= self.verlocity_queue.popleft()
        self.verlocity_queue_sum += self.dist_meter
        self.verlocity_queue.append(self.dist_meter)

    def update_distance(self, obj:Odometry):
        #init x,y values(happens only at start once)
        if self.x == None and self.y == None:
            self.x = abs(obj.pose.pose.position.x)
            self.y = abs(obj.pose.pose.position.y)

        #take last and new cords and calc distance
        else:
            new_x = abs(obj.pose.pose.position.x)
            new_y = abs(obj.pose.pose.position.y)
            self.dist_meter = abs(self.x - new_x) + abs(self.y - new_y)
            self.x = new_x
            self.y = new_y


    def publish_median_ticks(self):
        pub = rospy.Publisher('/autominy_msgs/Speed', Float64, queue_size=1)
        #rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            median = self.verlocity_queue_sum / self.verlocity_queue.maxlen
            pub.publish(median)
            print(median)
            #rate.sleep()



def main():
    rospy.init_node('asignment9_publisher', anonymous=True)
    node = MyNode()
    node.task9_b()

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

