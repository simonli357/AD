#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

class CameraHandler():
    # ===================================== INIT==========================================
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('CAMnod', anonymous=True)
        self.r = rospy.Rate(2.5)
        self.save_path = os.path.dirname(os.path.realpath(__file__))+'/rf2/'
        #create path
        os.makedirs(self.save_path, exist_ok=True)
        self.i = 0
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        rospy.spin()
        self.r.sleep()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imwrite(self.save_path+str(self.i)+".jpg", self.cv_image)
        print("Image saved: "+str(self.i))
        cv2.imshow("Frame preview", self.cv_image)
        cv2.waitKey(1)
        self.i+=1
        self.r.sleep()
        # cv2.imshow("Frame preview", self.cv_image)
        # key = cv2.waitKey(1)
    
            
if __name__ == '__main__':
    try:
        nod = CameraHandler()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass