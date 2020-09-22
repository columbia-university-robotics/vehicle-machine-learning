#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo


class VisualOdomRect:

    def __init__(self):
        rospy.init_node('visual_odom_rectifier')

        rospy.Subscriber("/rtabmap/visual_odom", Odometry, callback=self.odom_callback)
        self.rect_odom_pub = rospy.Publisher("/rtabmap/" + "visual_odom_rect", Odometry, queue_size=10)
        rospy.spin()

    def odom_callback(self, data):

        # correct the odometry axis
        pose_temp_z = data.pose.pose.position.z
        pose_temp_x = data.pose.pose.position.x
        pose_temp_y = data.pose.pose.position.y

        # based on experimental results
        # z -> -y
        # x -> -z
        # y -> x
        data.pose.pose.position.x = pose_temp_z
        data.pose.pose.position.y = -1 * pose_temp_x
        data.pose.pose.position.z = -1 * pose_temp_y

        self.rect_odom_pub.publish(data)


if __name__ == "__main__":
    VisualOdomRect()
