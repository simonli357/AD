#!/usr/bin/env python3

import rospy
import os
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    # Check if image path is passed
    if len(sys.argv) < 2:
        print("Usage: rosrun your_package image_publisher.py /path/to/image.jpg")
        return

    image_path = sys.argv[1]

    # Initialize ROS node
    rospy.init_node('image_publisher', anonymous=True)

    # Image publisher
    pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)

    # Check if file exists
    if not os.path.exists(image_path):
        rospy.logerr("Image file not found at path: {}".format(image_path))
        return

    # Read the image
    cv_image = cv2.imread(image_path)
    if cv_image is None:
        rospy.logerr("Failed to load image from path: {}".format(image_path))
        return

    # Create CvBridge
    bridge = CvBridge()
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        pub.publish(ros_image)
        rospy.loginfo("Published image to /camera/color/image_raw")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
