#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from collections import deque
from matplotlib import pyplot as plt
import numpy as np


class ImageRawRectifier:

    def __init__(self):
        rospy.init_node('image_raw_rectifier')

        self.deque = deque(maxlen=15)

        rospy.Subscriber("/" + rospy.get_param('rover_name') + "/camera/left/image_raw",
                         Image, callback=self.left_image_callback)
        rospy.Subscriber("/" + rospy.get_param('rover_name') + "/camera/right/image_raw",
                         Image, callback=self.right_image_callback)

        self.bridge = CvBridge()
        self.left_image_pub = rospy.Publisher("/" + rospy.get_param('rover_name') + "/camera/left/image_raw_rect",
                                              Image, queue_size=10)
        self.right_image_pub = rospy.Publisher("/" + rospy.get_param('rover_name') + "/camera/right/image_raw_rect",
                                               Image, queue_size=10)

        rospy.spin()

    def is_valid(self, image, verbose=False):
        # Convert image to HSV color space
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        n_white_pix = np.sum(image_gray>230)

        # Calculate histogram of saturation channel
        s = cv2.calcHist([image], [1], None, [256], [0, 256])

        # Calculate percentage of pixels with saturation >= p
        p = 0.05
        s_perc = np.sum(s[int(p * 255):-1]) / np.prod(image.shape[0:2])

        # Percentage threshold; above: valid image, below: noise
        self.deque.appendleft(s_perc)
        s_thr = sum(self.deque) / len(self.deque)

        if verbose:
            rospy.loginfo(s_thr)
            rospy.loginfo(s_perc)

        return s_perc <= s_thr and n_white_pix < 1000

    def right_image_callback(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        if self.is_valid(cv_image):
            self.right_image_pub.publish(data)

    def left_image_callback(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        if self.is_valid(cv_image):
            self.left_image_pub.publish(data)


if __name__ == "__main__":
    ImageRawRectifier()
