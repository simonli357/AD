#!/usr/bin/env python3

import rospy
import os
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def get_image_files_from_dir(directory):
    supported_exts = ('.jpg', '.jpeg', '.png', '.bmp', '.tiff')
    return sorted([os.path.join(directory, f) for f in os.listdir(directory)
                   if f.lower().endswith(supported_exts)])

def main():
    image_dir = "/media/slsecret/E624108524105B3F/Users/simon/Downloads/bfmc_data/base/testsets/rf0309b/images"

    if not os.path.isdir(image_dir):
        rospy.logerr("Provided path is not a directory: {}".format(image_dir))
        return

    image_files = get_image_files_from_dir(image_dir)
    if not image_files:
        rospy.logerr("No images found in directory: {}".format(image_dir))
        return

    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    index = 0
    cv_image = cv2.imread(image_files[index])
    if cv_image is None:
        rospy.logerr("Failed to load initial image.")
        return

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # Publish current image continuously
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        pub.publish(ros_image)
        rospy.loginfo_throttle(1, "Publishing: {}".format(os.path.basename(image_files[index])))

        # Display the image in a window
        cv2.imshow("Image Viewer (Press 'a'=prev, 'd'=next, 'q'=quit)", cv_image)
        key = cv2.waitKey(1) & 0xFF  # Non-blocking

        if key == ord('d'):
            index = (index + 1) % len(image_files)
            cv_image = cv2.imread(image_files[index])
        elif key == ord('a'):
            index = (index - 1) % len(image_files)
            cv_image = cv2.imread(image_files[index])
        elif key == ord('q') or key == 27:
            break

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
