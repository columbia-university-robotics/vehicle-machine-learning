#!/usr/bin/env python
# Converts laser scan to planar distance
# and segments into obstalces and just hills

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, Imu, Image
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
import cv2
from copy import deepcopy
import math
import scipy.stats
import numpy as np


def to_hsv(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)


class ObjectDetector:
    """Handles the logic of detecting the cubesat, the base, and the base front.
    """
    # color that ALWAYS occurs in a base connected component and 
    # NEVER occurs in a cubesat or rover connected component
    def __init__(self):
        pass

    base_key = ((5, 100, 100), (15, 255, 255))

    # color range of base, cubesat and rover
    object_ranges = [
        ((10, 25, 70), (60, 255, 255))
    ]

    # Base Front Color Range
    front_ranges = [
        ((75, 50, 50), (150, 255, 255))
    ]

    def segment(self, hsv_img):

        base_mask = self.detect_base(hsv_img)
        base_mask[base_mask > 0] = 100
        base_front_mask = self.detect_base_front(hsv_img)
        base_front_mask[base_front_mask > 0] = 150
        cubesat_mask = self.detect_cubesat(hsv_img)
        cubesat_mask[cubesat_mask > 0] = 50
        rover_mask = self.detect_rover(hsv_img)
        rover_mask[rover_mask > 0] = 00

        return base_mask + base_front_mask + cubesat_mask + rover_mask

    def detect_base(self, hsv_img):
        mask = self.comp_mask(hsv_img, self.object_ranges, self.base_key, inverse_key=False)
        return mask

    def detect_base_front(self, hsv_img):
        mask = self.comp_mask(hsv_img, self.front_ranges, max_width_height_ratio=2)
        return mask

    def detect_cubesat(self, hsv_img):
        mask = self.comp_mask(hsv_img, self.object_ranges, self.base_key, inverse_key=True)
        return mask

    def detect_rover(self, hsv_img):
        mask = self.comp_mask(hsv_img, self.object_ranges, self.base_key, inverse_key=True)
        return mask

    def comp_mask(self, hsv_img, ranges, key=None, inverse_key=False, max_width_height_ratio=None):
        mask = None
        for r in ranges:
            new_mask = cv2.inRange(hsv_img, r[0], r[1])
            if mask is None:
                mask = new_mask
            else:
                mask = cv2.bitwise_or(mask, new_mask)
        # now grow mask to encompass some background and connect things up
        kernel = np.ones((3, 3))
        mask = cv2.dilate(mask, kernel, iterations=4)
        # now remove mask elements of outlier low pixels
        connected_comps = cv2.connectedComponentsWithStats(mask, 8)
        if connected_comps[0] == 1:  # only background detected (i.e. no component)
            return mask  # return early
        labels = connected_comps[1]
        pixel_counts = [i[4] for i in connected_comps[2][1:]]
        avg_pixel_count = np.mean([max(pixel_counts), np.mean(pixel_counts)])
        min_count = min(pixel_counts)
        if avg_pixel_count / min_count < 4:  # no indication of "too small bits"
            threshold = 0
        else:
            threshold = min_count + (5 if (avg_pixel_count - min_count) / 4 < 5 else (avg_pixel_count - min_count) / 4)
        for idx, stats in enumerate(connected_comps[2]):
            if stats[4] < threshold:
                out_mask = np.array(labels, dtype=np.uint8)
                out_mask[labels == idx] = 0.0
                out_mask[labels != idx] = 1.0
                mask = cv2.bitwise_and(mask, out_mask)
        # Now strip any connected components that don't have key color if key is specified
        if key is not None or max_width_height_ratio is not None:
            conn_comps = cv2.connectedComponentsWithStats(mask, 8)
            labels = conn_comps[1]
            # print('Pre-cut # comps:', conn_comps[0] - 1)
            for lab in range(1, conn_comps[0]):
                do_cut = False
                out_mask = np.array(labels, dtype=np.uint8)
                out_mask[labels == lab] = 1
                out_mask[labels != lab] = 0
                if key is not None:
                    cropped = cv2.bitwise_and(hsv_img, hsv_img, mask=out_mask)
                    new_mask = cv2.inRange(cropped, key[0], key[1])
                    no_match = np.all(new_mask == 0)
                    do_cut = do_cut or ((no_match and not inverse_key) or (not no_match and inverse_key))
                if max_width_height_ratio is not None:
                    _, _, width, height, _ = conn_comps[2][lab]
                    if (width / height) > max_width_height_ratio:
                        do_cut = True
                if do_cut:  # colors that should or shouldn't be there
                    cutout = np.array(out_mask)
                    cutout[out_mask == 0] = 1
                    cutout[out_mask > 0] = 0
                    mask = cv2.bitwise_and(mask, mask, mask=cutout)  # cut out this component
        comps = cv2.connectedComponentsWithStats(mask, 8)
        # print('Final # connected components:', comps[0] - 1)
        return mask


class ImageSegmenter:

    """ Takes in stereo camera depth image and produces estimates of detected
    obstacles
    """

    def __init__(self):

        rospy.init_node("image_segmenter")

        # TODO: remove hard coded constant
        self.NUM_BUCKETS = 4
        self.bridge = CvBridge()
        self.detector = ObjectDetector()

        img_topic = rospy.get_param('~image_topic', '/stereo/image_proc_right/image_rect_color')
        depth_img_topic = rospy.get_param('~image_topic', '/stereo/image_proc_right/image_rect_color')
        pub_topic = rospy.get_param('~tagged_image_topic', '/segmented_image')

        # subscriber and publishers
        self.image_mask_pub = rospy.Publisher(pub_topic, Image, queue_size=4)
        rospy.Subscriber(img_topic, Image, callback=self.segment_callback)
        # rospy.Subscriber(depth_img_topic, Image, callback=self.depth_callback)

        rospy.spin()

    def segment_callback(self, data):

        img = self.bridge.imgmsg_to_cv2(data, "passthrough")
        hsv_img = to_hsv(img)
        segmented_frame = self.detector.segment(hsv_img)

        try:
            self.image_mask_pub.publish(self.bridge.cv2_to_imgmsg(segmented_frame, "mono8"))
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "32FC1")
        depths = np.array(img, dtype=np.float32)
        width = len(depths[0])
        buckets = [[] for i in range(self.NUM_BUCKETS)]
        for row in depths:
            for i, val in enumerate(row):
                if not np.isnan(val):
                    deg = self._idx_to_degree(i, len(depths[0]))
                    idx = self._deg_to_bucket(deg, self.NUM_BUCKETS)
                    buckets[idx].append(val)


if __name__ == "__main__":
    ImageSegmenter()
