#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

#Quelle für ROS Klassen Struktur
#https://stackoverflow.com/questions/37373211/update-the-global-variable-in-rospy


class MyNode():
    def __init__(self):
        #paras
        k1, k2, t1, t2, k3 = 0, 0, 0, 0, 0
        fx, cx, fy, cy = 0, 0, 0, 0

        #pubs

        #subs
        rospy.Subscriber('/sensors/camera/infra1/camera_info', CameraInfo  , self.extract_paras)
        rospy.Subscriber('/sensors/camera/infra1/image_rect_raw', Image  , self.img_callback)

    #4-2
    # extract the intrinsic parameters fx,fy,cx,cy and the distortion coefficients
    # k1,k2,t1,t2,k3 from the /sensors/camera/infra1/camera info topic.
    def extract_paras(self, cameraInfo : CameraInfo):
        self.k1, self.k2, self.t1, self.t2, self.k3 = cameraInfo.D
        self.fx, _, self.cx, _, self.fy, self.cy, _, _, _ = cameraInfo.K

        #ZUM TESTEN
        #PRINT 4-2 
        #print(f'k1:{self.k1}, k2:{self.k2}, t1:{self.t1}, t2:{self.t2}, k3:{self.k3}')
        #print(f'fx:{self.fx}, cx:{self.cx}, fy:{self.fy}, cy:{self.cy}\n\n')

    #4-3
    def extract_image(self, ros_img : Image):
        cv_img = CvBridge().imgmsg_to_cv2(ros_img, desired_encoding='passthrough')
        _ , binary_img = cv.threshold(cv_img, 243, 255, cv.THRESH_BINARY)

        cv.rectangle(binary_img,(0,0),(640,50),(0,0,0),-1)      #top rect
        cv.rectangle(binary_img,(550,50),(640,85),(0,0,0),-1)   #right rect
        cv.rectangle(binary_img,(0,350),(640,480),(0,0,0),-1)   #bottom rect
        cv.rectangle(binary_img,(250,250),(575,480),(0,0,0),-1) #car rect

        return binary_img

    #4-4
    def find_white_pixels(self, binary_img):
        #in order: x1,y1 - x2,y2
        all_regions = (
            (250,110,285,130), #top_left_area
            (400,100,435,120), #top_right_area
            (225,150,270,175), #middle_left_area
            (425,140,465,160), #middle_right_area
            (180,220,230,255), #bottom_left_area
            (475,210,535,235)  #bottom_right_area
        )
         
        all_medians = []
        for region in all_regions:
            x1,y1,x2,y2 = region
            median = [0, 0]
            whitepixel_count = 0
            for x in range(x1,x2):
                for y in range(y1,y2):
                    if binary_img[y,x] == 255:
                        median[0] += x
                        median[1] += y
                        whitepixel_count += 1

            median[0] /= whitepixel_count
            median[1] /= whitepixel_count
            all_medians += [median]

        #in order: tl, tr, ml, mr, bl, br and (x,y)
        return all_medians

    #4-5
    def compute_extrinsic_paras(self, camera_2d):
        world_3d = np.array([
            [1.1,  0.2, 0], #top left
            [1.1, -0.2, 0], #top right
            [0.8,  0.2, 0], #mid left
            [0.8, -0.2, 0], #mid right
            [0.5,  0.2, 0], #bottom left
            [0.5, -0.2, 0]  #bottom right
        ])
        camera_2d = np.array(camera_2d)
        intrinsic = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])
        distortion = np.array([self.k1, self.k2, self.t1, self.t2, self.k3])

        _, rvec, tvec = cv.solvePnP(world_3d, camera_2d, intrinsic, distortion)

        return rvec, tvec

    #4-6
    def compute_homogene_matrix(self, rvec, tvec):
        rotation_matrix, _ = cv.Rodrigues(rvec)

        # [R(3x3)  t(3x1)]
        # [0(1x3)     1  ]
        homogene_matrix = np.concatenate((rotation_matrix, tvec), axis=1)
        homogene_matrix = np.vstack( (homogene_matrix, np.array([0,0,0,1])) )

        # [R^⁻1  -R^⁻1*t]
        # [0        1   ]
        inverse_rotation_matrix = rotation_matrix.transpose()
        inverse_tvec = np.matmul(-inverse_rotation_matrix, tvec)
        inverse = np.concatenate((inverse_rotation_matrix, inverse_tvec), axis=1)
        inverse = np.vstack( (inverse, np.array([0,0,0,1])) )

        return homogene_matrix, inverse

    def img_callback(self, ros_img : Image):
        binary_img  = self.extract_image(ros_img)
        all_medians = self.find_white_pixels(binary_img)
        rvec, tvec  = self.compute_extrinsic_paras(all_medians)
        hom, inv    = self.compute_homogene_matrix(rvec,tvec)


        #ZUM TESTEN
        #PRINT 4-3
        #cv.imshow("Display window", binary_img)
        #cv.waitKey(3) 
        #or
        #from numpy.lib.type_check import imag
        #import skimage
        #from skimage import io
        #io.imshow(binary_img)
        #io.show()

        #PRINT 4-4
        #print('in order: tl, tr, ml, mr, bl, br and (x,y)')
        #print(f'ALL MEDIANS: {all_medians}\n\n')

        #PRINT 4-5
        #print(f'rotation vector:\n{rvec}\ntranslation vector:\n{tvec}\n\n')

        #PRINT4-6
        #print(f'Homogeneous Matrix:\n{hom} \nInverse:\n{inv}\n\n')

    def publish(self):
        while not rospy.is_shutdown():
            pass



def main():
    rospy.init_node('Camera_Subscriber', anonymous=True)
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

