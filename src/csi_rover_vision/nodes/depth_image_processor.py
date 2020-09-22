#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import numpy as np
import message_filters
import tf


class DepthImageProcessor:

    def __init__(self):

        self.HOME_BASE_KEY = 1

        rospy.init_node('depth_image_processor')
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        # subscribe to depth image and segmentation result
        self.base_pose_pub = rospy.Publisher("/base_station/pose", PoseStamped, queue_size=3)
        rospy.Subscriber("/stereo/depth_image", Image, callback=self.depth_image_callback)
        rospy.Subscriber("/segmented_image", Image, callback=self.segmentat_image_callback)
        self.odom_frame = "odom"
        self.camera_frame = "scout_1_tf/camera_link"

        self.depth_image = None

        print("started node")

        rospy.spin()

    def depth_image_callback(self, depth_image):

        try:
            cv_mat_depth = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        except CvBridgeError, e:
            raise e

        self.depth_image = cv_mat_depth
        # # Convert the depth image to a Numpy array
        # self.depth_image = np.array(cv_mat_depth, dtype=np.float32)

    def segmentat_image_callback(self, segmentation_image):

        try:
            cv_mat_seg = self.bridge.imgmsg_to_cv2(segmentation_image, desired_encoding="mono8")
        except CvBridgeError, e:
            raise e

        if self.depth_image is not None:

            cv_mat_seg = np.array(cv_mat_seg)
            cv_mat_seg[cv_mat_seg != self.HOME_BASE_KEY] = 0
            cv_mat_seg[cv_mat_seg > 0] = 1
            masked_depth = cv2.bitwise_and(self.depth_image, self.depth_image, mask=cv_mat_seg)

            # convert depth mask to numpy and clean
            np_array = np.array(masked_depth).flatten()
            where_are_NaNs = np.isnan(np_array)
            np_array[where_are_NaNs] = 0

            count = np.count_nonzero(np_array)
            sum = np_array.sum()
            dist = sum / count

            obj_pose = PoseStamped()
            obj_pose.header = Header()
            obj_pose.header.frame_id = self.camera_frame
            obj_pose.pose.position.x = dist
            obj_pose.pose.position.y = 0
            obj_pose.pose.position.z = 0

            final_pose = self.listener.transformPose(self.odom_frame, obj_pose)
            self.base_pose_pub.publish(final_pose)
            # print(obj_pose)
            # print(final_pose)

            # cv2.imshow("masked_data", masked_depth)
            # cv2.waitKey(0)


if __name__ == "__main__":
    DepthImageProcessor()


