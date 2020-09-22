#!/usr/bin/env python

"""

Global path planner script for the CSI
NASA robotics challenge.

Used in the navigation package.

Author: Neil Nie
Craeted: 2020-04-26
Copyright: Columbia Space Initiative

"""

import math

from tf import transformations
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Path
import rospy

from path_generator import PathGenerator


def convert_to_quaternion(yaw, pitch, roll):

    array = transformations.quaternion_from_euler(yaw, pitch, roll)
    quart = Quaternion()
    quart.x = array[0]
    quart.y = array[1]
    quart.z = array[2]
    quart.w = array[3]

    return array


def path_from_coordinates(waypoints):

    path = Path()
    path.header = Header()
    path.header.frame_id = "map"
    path.poses = []
    count = 0
    # convert x,y coordinates to PoseStamped
    for waypoint in waypoints:

        pose = PoseStamped()
        pose.header = Header()
        pose.header.seq = count
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = 0

        path.poses.append(pose)
        count += 1

    # update the orientation of the goals
    for i in range(len(path.poses) - 1):

        pose = path.poses[i]
        next_pose = path.poses[i+1]

        # print(pose.pose.position)
        # print(next_pose.pose.position)
        # print("-----------------------")

        d_x = next_pose.pose.position.x - pose.pose.position.x
        d_y = next_pose.pose.position.y - pose.pose.position.y

        yaw = math.atan2(d_y, d_x)

        orientation = convert_to_quaternion(0, 0, yaw)
        path.poses[i].pose.orientation.x = orientation[0]
        path.poses[i].pose.orientation.y = orientation[1]
        path.poses[i].pose.orientation.z = orientation[2]
        path.poses[i].pose.orientation.w = orientation[3]

    orientation = convert_to_quaternion(math.pi / 2, 0, 0)
    path.poses[-1].pose.orientation.x = orientation[0]
    path.poses[-1].pose.orientation.y = orientation[1]
    path.poses[-1].pose.orientation.z = orientation[2]
    path.poses[-1].pose.orientation.w = orientation[3]

    return path


class GlobalPlanner:

    def __init__(self):

        rospy.init_node('global_planner')
        self.path_type = rospy.get_param('path_type', 1)
        
        self.path_pub = rospy.Publisher("/navigation/global_planner/path", Path, queue_size=10)
        self.debug = rospy.Publisher("/one_pose", PoseStamped, queue_size=5)
        self.debug2 = rospy.Publisher("/one_pose_2", PoseStamped, queue_size=5)

        # path generator class
        self.path_gen = PathGenerator()
        self.path = path_from_coordinates(self.path_gen.generator_rounded_path())

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

          self.path_pub.publish(self.path)
          #self.debug.publish(self.path.poses[8])
          #self.debug2.publish(self.path.poses[10])

          rate.sleep()


if __name__ == '__main__':

    try:
        GlobalPlanner()
    except rospy.ROSInternalException:
        pass

