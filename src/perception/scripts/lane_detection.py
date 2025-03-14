#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LanePreprocessingVisualizer:
    def __init__(self):
        rospy.init_node('lane_preprocessing_comparison_node')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('frame', frame)
        # Resize frame for better visualization
        # frame = cv2.resize(frame, (640, 360))

        # 1. Grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #blur
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # 2. CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        clahe_applied = clahe.apply(gray)

        # 3. Sobel X (horizontal gradient)
        sobelx = cv2.Sobel(clahe_applied, cv2.CV_64F, 1, 0, ksize=3)
        sobelx_abs = np.uint8(np.absolute(sobelx))

        # 4. Adaptive Thresholding (on CLAHE image)
        adaptive_thresh = cv2.adaptiveThreshold(
            clahe_applied, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            257, -40
        )
        
        adaptive_thresh2 = cv2.adaptiveThreshold(
            clahe_applied, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            199, -40
        )
        
        adaptive_thresh_gray = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            199, -20
        )

        # 5. Gaussian Blur + Canny Edge Detection
        blurred = cv2.GaussianBlur(clahe_applied, (5, 5), 0)
        canny = cv2.Canny(blurred, 50, 150)
        
        # 1. HLS Color Space
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        l_channel = hls[:, :, 1]
        clahe_l = clahe.apply(l_channel) # Apply CLAHE on L channel, which is more robust to lighting changes.

        # 3. Refined Adaptive Threshold + Morph + ROI
        kernel = np.ones((3, 3), np.uint8)
        adaptive_refined = cv2.morphologyEx(adaptive_thresh, cv2.MORPH_OPEN, kernel)
        adaptive_refined = cv2.morphologyEx(adaptive_refined, cv2.MORPH_CLOSE, kernel)

        mask_roi = np.zeros_like(adaptive_refined)
        mask_roi[int(adaptive_refined.shape[0] / 2.5):, :] = 255
        # adaptive_refined = cv2.bitwise_and(adaptive_refined, mask_roi)
        
        adaptive_refined2 = cv2.morphologyEx(adaptive_thresh2, cv2.MORPH_OPEN, kernel)
        adaptive_refined2 = cv2.morphologyEx(adaptive_refined2, cv2.MORPH_CLOSE, kernel)
        mask_roi2 = np.zeros_like(adaptive_refined2)
        mask_roi2[int(adaptive_refined2.shape[0] / 2.5):, :] = 255
        # adaptive_refined2 = cv2.bitwise_and(adaptive_refined2, mask_roi2)
        
        adaptive_refined_gray = cv2.morphologyEx(adaptive_thresh_gray, cv2.MORPH_OPEN, kernel)
        adaptive_refined_gray = cv2.morphologyEx(adaptive_refined_gray, cv2.MORPH_CLOSE, kernel)
        mask_roi_gray = np.zeros_like(adaptive_refined_gray)
        mask_roi_gray[int(adaptive_refined_gray.shape[0] / 2.5):, :] = 255
        # adaptive_refined_gray = cv2.bitwise_and(adaptive_refined_gray, mask_roi_gray)

        def resize_disp(img): return cv2.resize(img, (320, 180))
        
        # Resize all images for side-by-side display
        gray_disp     = cv2.resize(gray, (320, 180))
        clahe_disp    = cv2.resize(clahe_applied, (320, 180))
        sobel_disp    = cv2.resize(sobelx_abs, (320, 180))
        adaptive_disp = cv2.resize(adaptive_thresh, (320, 180))
        canny_disp    = cv2.resize(canny, (320, 180))
        adaptive_disp2 = cv2.resize(adaptive_thresh2, (320, 180))
        adaptive_disp_gray = cv2.resize(adaptive_thresh_gray, (320, 180))
        adaptive_refined_disp = resize_disp(adaptive_refined)
        adaptive_refined_disp2 = resize_disp(adaptive_refined2)
        adaptive_refined_disp_gray = resize_disp(adaptive_refined_gray)

        # Stack all preprocessing results in 2 rows
        row2 = np.hstack((gray_disp, clahe_disp, canny_disp))
        row1 = np.hstack((adaptive_disp, adaptive_disp2, adaptive_disp_gray))
        row3 = np.hstack((adaptive_refined_disp, adaptive_refined_disp2, adaptive_refined_disp_gray))
        final_vis = np.vstack((row1, row2, row3))

        # Add labels (optional - can annotate each image)
        cv2.putText(row2, "Gray   CLAHE   Sobel X", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1)
        cv2.putText(row1, "Adaptive Thresh   Canny Edge", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1)
        cv2.putText(row3, "CLAHE L   Color Mask   Adaptive Refined", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1)

        # 1. Global Histogram Equalization + Custom Threshold + Morph
        global_eq = cv2.equalizeHist(gray)
        global_blur = cv2.GaussianBlur(global_eq, (5, 5), 0)

        hist = cv2.calcHist([global_blur], [0], None, [gray.shape[0]], [0, 256])
        minVal, maxVal, _, _ = cv2.minMaxLoc(hist)
        thresh_val = np.clip(int(maxVal - 75), 30, 200)

        _, global_thresh = cv2.threshold(global_blur, thresh_val, 255, cv2.THRESH_BINARY)

        kernel3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        kernel5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        global_morph = cv2.morphologyEx(global_thresh, cv2.MORPH_OPEN, kernel3)
        global_morph = cv2.morphologyEx(global_morph, cv2.MORPH_CLOSE, kernel5)

        # 2. CLAHE + Otsu + Morph
        clahe_otsu_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        clahe_otsu_applied = clahe.apply(clahe_otsu_blur)
        _, otsu_thresh = cv2.threshold(clahe_otsu_applied, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        otsu_morph = cv2.morphologyEx(otsu_thresh, cv2.MORPH_CLOSE, kernel)
        otsu_morph = cv2.morphologyEx(otsu_morph, cv2.MORPH_OPEN, kernel)

        # Resize for display
        global_morph_disp = resize_disp(global_morph)
        otsu_morph_disp = resize_disp(otsu_morph)

         # 3. OLD Method - Histogram-based simple threshold (no CLAHE, no morph)
        hist_old = cv2.calcHist([gray], [0], None, [gray.shape[0]], [0, 256])
        _, maxVal_old, _, _ = cv2.minMaxLoc(hist_old)
        threshold_old = np.clip(int(maxVal_old - 75), 30, 200)

        _, old_thresh = cv2.threshold(gray, threshold_old, 255, cv2.THRESH_BINARY)
        old_disp = resize_disp(old_thresh)
        
        # Create a placeholder for 3rd image in row4 (optional or duplicate any if needed)
        row4 = np.hstack((global_morph_disp, otsu_morph_disp, old_disp))

        # Update final visual stack
        final_vis = np.vstack((row1, row2, row3, row4))

        # (Optional) Add labels if needed
        cv2.putText(row4, "GlobalHist+Morph   CLAHE+Otsu+Morph", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1)
        
        # Show the final stacked visualization
        cv2.imshow("Preprocessing Comparison", final_vis)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        LanePreprocessingVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
