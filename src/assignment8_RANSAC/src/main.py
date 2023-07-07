#!/usr/bin/env python
from re import X
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
import random, time

class MyNode():
    def __init__(self):
        #paras
        self.global_n_counter =1
        self.global_total_ransac_runs =1

        #pubs
        self.pub_img = rospy.Publisher('/mypub/img', Image, queue_size=10)
        self.pub_line1=rospy.Publisher('/mypub/line1',String,queue_size = 10)
        self.pub_line2 = rospy.Publisher('/mypub/line2', String, queue_size=10)

        #subs
        rospy.Subscriber('/sensors/camera/infra1/image_rect_raw', Image  , self.img_callback)

    def img_callback(self, ros_img : Image):
        print(f'Durschnittliche Anzahl an Iterationen von ransac:{self.global_n_counter/self.global_total_ransac_runs}')

        #variables
        threshold = 30
        n_tries = 40
        proportionOfInliners_1 = 0.5
        proportionOfInliners_2 = 0.7

        #img cromp. paints right side of img black 
        cv_img, binary_img = self.get_cv_and_binary_img(ros_img)
        binary_img = cv.line(binary_img, (640,310), (450,135), (0, 0, 0), 160)


        #ransac for line 1
        wList = self.white_image_points_to_list(binary_img)
        inliners = self.ransac(wList, n_tries, threshold, proportionOfInliners_1,cv_img)

        #get line 1
        p1, p2 = inliners
        line1 = self.points_to_line(p1,p2)

        #draw line on original img
        cv_img = cv.line(cv_img, p1, p2, (0, 0, 0), threshold)

        #ransac for line 2
        #draw line on binary img (twice as much)
        binary_img = cv.line(binary_img, p1, p2, (0, 0, 0), threshold*4)
        wList = self.white_image_points_to_list(binary_img)
        inliners = self.ransac(wList, n_tries, 15, proportionOfInliners_2,cv_img)

        #get line 1
        p1, p2 = inliners
        line2 = self.points_to_line(p1,p2)

        #draw line on original img 
        cv_img = cv.line(cv_img, p1, p2, (0, 0, 0), threshold)
        
        
        #pub
        self.pub_img.publish((CvBridge().cv2_to_imgmsg(cv_img)))
        self.pub_line1.publish(f'm:{line1[0]}, b:{line1[1]}')
        self.pub_line2.publish(f'm:{line2[0]}, b:{line2[1]}')


        







    #
    def unused_functions():
        #TODO might be useless 
        #not used as way to slow
        def white_image_points_to_dict(self, binary_img):
            max_y = len(binary_img)
            max_x = len(binary_img[0])
            dict = {}
            for y in range(max_y):
                for x in range(max_x):
                    if binary_img[y,x] == 255:
                        dict[(y,x)] = [y,x]
            return dict

        #TODO might be useless
        #not used as way to slow
        def get_2_random_points_from_dict(self, dict):
            p1 = random.choice(list(dict.values()))
            p2 = random.choice(list(dict.values()))
            return (p1[0],p1[1]), (p2[0],p2[1])

        #TODO might be useless
        def shift_line(self, m,b, value):
            upper_line = (m, b + value + value*m)
            lower_line = (m, b - value - value*m)
            return upper_line, lower_line

        #TODO might be useless
        def shift_line_points(self,p1,p2,value):
            upper_p1, upper_p2 = (p1[0]+value, p1[1]+value), (p2[0]+value, p2[1]+value)
            lower_p1, lower_p2 = (p1[0]-value, p1[1]-value), (p2[0]-value, p2[1]-value)
            return upper_p1, upper_p2, lower_p1, lower_p2

        #TODO might be useless
        def list_to_dict(self, list):
            return {i:0 for i in list}
        
        #TODO useless
        def line_points_to_rect(self, p1,p2,width):
            y1, x1 = p1[0], p1[1]
            y2, x2 = p2[0], p2[1]
            #find upper point
            if y1 < y2:
                y1, x1, y2, x2 = y2, x2,  y1, x1
            bottom_left_point = (y1+width,x1-width)
            top_right_point = (y2-width,x2+width)
            #print(f'bl:{bottom_left_point}, tr:{top_right_point}')
            return bottom_left_point, top_right_point

        #TODO useless
        def test_if_point_in_rect(self,bl, tr, point):
            y1, x1 = bl[0], bl[1]
            y2, x2 = tr[0], tr[1]
            y, x = point[0], point[1]
            if(x1 <= x2 and y1 >= y2 or True):
                if x1 < x < x2 and y1 > y > y2:
                    #print(f'bl:{bl}, tr:{tr}, point:{point}')
                    return True
                else:
                    return False
            raise ValueError('rect in wronge format')

        #TODO useless
        def line_get_y(self, m,b,x):
            return int(round(m*x+b))
        
        #TODO useless
        def line_get_x(self, m,b,y):
            if m == 0:
                return -1
            return int(round((y-b)/m))

        #TODO old/ slow useless
        def extend_line_to_img_boundaries(self,m,b,img):
            xMinAxis = 0
            yMinAxis = 0
            yMaxAxis = len(img)
            xMaxAxis = len(img[0])

            bl_x = self.line_get_x(m,b,yMinAxis)
            bl_y = self.line_get_y(m,b,xMinAxis)
            tr_x = self.line_get_x(m,b,yMaxAxis)
            tr_y = self.line_get_y(m,b,xMaxAxis)

            two_points = []

            #print(f'bl_x:{bl_x}, bl_y:{bl_y}, tr_x:{tr_x}, tr_y:{tr_y}')

            if xMaxAxis >= bl_x >= xMinAxis:
                two_points+= [(0,bl_x)]

            if yMaxAxis >= bl_y >= yMinAxis:
                two_points+= [(bl_y,0)]

            if xMaxAxis >= tr_x >= xMinAxis:
                two_points+= [(yMaxAxis,tr_x)]

            if yMaxAxis >= tr_y >= yMinAxis:
                two_points+= [(tr_y,xMaxAxis)]

            return two_points
        

    #elementare functionen

    def get_cv_and_binary_img(self, ros_img : Image):
        cv_img = CvBridge().imgmsg_to_cv2(ros_img, desired_encoding='passthrough')
        _ , binary_img = cv.threshold(cv_img, 243, 255, cv.THRESH_BINARY)
        return cv_img, binary_img

    def white_image_points_to_list(self, binary_img):
        return cv.findNonZero(binary_img)

    def get_2_random_points_from_list(self, _list):
        p1 = random.choice(_list)
        p2 = random.choice(_list)
        return (p1[0,0],p1[0,1]), (p2[0,0],p2[0,1])

    def points_to_line(self, p1 , p2):
        y1, x1 = p1[0], p1[1]
        y2, x2 = p2[0], p2[1]
        #sonderfall
        if (x1-x2 == 0):
            m = 0
            b = y1
            return (m,b)
        m = (y2-y1)/(x2-x1)
        b = m*(-x1)+y1
        return (m,b)

    def distance_of_point_to_line(self, pl1,pl2,p_3):
        p1 = np.asarray([pl1[1],pl1[0]])
        p2 = np.asarray([pl2[1],pl2[0]])
        p3 = np.asarray([p_3[1],p_3[0]])
        return np.abs(np.cross(p2-p1, p1-p3)) / np.linalg.norm(p2-p1)

    def get_intersection_of_two_lines(self, line1, line2):
        """ 
        Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
        a1: [x, y] a point on the first line
        a2: [x, y] another point on the first line
        b1: [x, y] a point on the second line
        b2: [x, y] another point on the second line
        """
        a1, a2 = line1
        b1, b2 = line2

        s = np.vstack([a1,a2,b1,b2])        # s for stacked
        h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
        l1 = np.cross(h[0], h[1])           # get first line
        l2 = np.cross(h[2], h[3])           # get second line
        x, y, z = np.cross(l1, l2)          # point of intersection
        if z == 0:                          # lines are parallel
            return (float('inf'), float('inf'))
        return (x/z, y/z)

    def get_intersection_of_img_boundaries(self, p1, p2, max_y, max_x):
        test_line = ([p1[1],p1[0]], [p2[1],p2[0]])

        xMinAxis = 0
        yMinAxis = 0
        yMaxAxis = max_y
        xMaxAxis = max_x

        bl_p = [xMinAxis, yMaxAxis]
        tl_p = [xMinAxis, yMinAxis]
        tr_p = [xMaxAxis, yMinAxis]
        br_p = [xMaxAxis, yMaxAxis]

        line_bottom = (bl_p, br_p)
        line_left = (bl_p, tl_p)
        line_top = (tl_p, tr_p)
        line_right = (tr_p, br_p)

        p_1 = self.get_intersection_of_two_lines(line_bottom, test_line)
        p_2 = self.get_intersection_of_two_lines(line_left, test_line)
        p_3 = self.get_intersection_of_two_lines(line_top, test_line)
        p_4 = self.get_intersection_of_two_lines(line_right, test_line)
        points = [p_1, p_2, p_3, p_4]

        two_result_points = []
        #test if in boundaries
        for point in points:
            x, y = point
            if xMaxAxis >= x >= xMinAxis and yMaxAxis >= y >= yMinAxis:
                two_result_points += [(int(round(y)), int(round(x)))]
        
        #if error
        if len(two_result_points) != 2:
            return [(0,0),(0,0)]

        return two_result_points


    #ransac functionen

    def get_maybeInlinser(self, wList):
        p1, p2 = self.get_2_random_points_from_list(wList)
        return (p1,p2)

    def get_maybeModel(self, maybeInlinser, max_y, max_x):
        p1, p2 = maybeInlinser
        bl_intersection, tr_intersection = self.get_intersection_of_img_boundaries(p1,p2, max_y, max_x)
        return (bl_intersection, tr_intersection)

    def point_nextTo_maybeModel(self, maybeModel, point, threshold):
        p1, p2 = maybeModel
        return threshold >= self.distance_of_point_to_line(p1,p2,point)

    def ransac(self,data_wList,n_repeat,t_thresholdDistance, d_proportionOfInliners,cv_img, s_randomPoints=2):
        counter = 0
        max_y, max_x = len(cv_img[0]), len(cv_img)
        for _ in range(n_repeat):
            counter += 1
            maybeInliners = self.get_maybeInlinser(data_wList)
            maybeModel = self.get_maybeModel(maybeInliners, max_y, max_x)
            alsoInliners_counter = 0

            #for testing
            #cv.line(cv_img, maybeInliners[0], maybeInliners[1], (0, 0, 255), 10)
            #cv.line(cv_img, maybeModel[0], maybeModel[1], (0, 255, 0), 2)

            for wPoint in data_wList:
                if self.point_nextTo_maybeModel(maybeModel, (wPoint[0,0], wPoint[0,1]), t_thresholdDistance):
                    alsoInliners_counter+=1
                    #for testing
                    #cv.line(cv_img, (wPoint[0,0], wPoint[0,1]), (wPoint[0,0], wPoint[0,1]), (0, 255, 0), 10)
                    #cv.imshow("Display window", cv_img)
                    #cv.waitKey(3) 

            if (alsoInliners_counter/len(data_wList)) >= d_proportionOfInliners:
                #print(alsoInliners_counter/len(data_wList), alsoInliners_counter, len(data_wList), d_proportionOfInliners)
                self.global_n_counter += counter
                self.global_total_ransac_runs +=1
                return maybeModel

        self.global_n_counter += counter
        self.global_total_ransac_runs +=1
        return (0,0), (0,0)

def main():
    rospy.init_node('RANSAC', anonymous=True)
    node = MyNode()

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

