#!/usr/bin/env python

import rospy
import os
import math
from collections import deque
from srcp2_msgs.msg import VolSensorMsg
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from srcp2_msgs.srv import Qual1ScoreSrv
from tf import transformations
from gazebo_msgs.msg import ModelStates


class PointScoreNode:

    def __init__(self):

        rospy.init_node('point_score_node', anonymous=True)
        rospy.Subscriber("/scout_1/volatile_sensor", VolSensorMsg, callback=self.volatile_sensor_callback)
        rospy.Subscriber("/scout_1/ekf_odom", Odometry, callback=self.rover_pose_callback)

        self.current_pose = None
        self.reported_volatiles = []

        rospy.spin()

    def volatile_sensor_callback(self, data):

        if self.current_pose is not None:

            self.reported_volatiles.append(data.vol_index)

            rospy.wait_for_service("/vol_detected_service", 2)
            rospy.loginfo("[REPORT]: Waiting to report detected volatile")
            try:
                report_volatile = rospy.ServiceProxy("/vol_detected_service", Qual1ScoreSrv)
                response = report_volatile(self.current_pose.pose.pose.position, data.vol_type)
                print(response)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

    def rover_pose_callback(self, data):
        self.current_pose = data


if __name__ == '__main__':
    try:
        PointScoreNode()
    except rospy.ROSInterruptException:
        pass
