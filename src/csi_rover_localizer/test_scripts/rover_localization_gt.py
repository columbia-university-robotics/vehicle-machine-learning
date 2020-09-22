#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Pose


class RoverLocalizationGT:

    def __init__(self):

        self.set_init_pose = 0
        self.init_pose = None
        self.curr_pose = None
        self.init_twist = None
        self.curr_twist = None

        self.rover_name = rospy.get_param("rover_name", "scout_1")

        rospy.Subscriber("/gazebo/model_states", ModelStates, callback=self.model_states_callback)

        self.gt_pub = rospy.Publisher("/debugging/gt_odom", Odometry, queue_size=5)
        self.init_pose_pub = rospy.Publisher("/debugging/init_pose", Pose, queue_size=5)

        rospy.init_node('rover_localization_gt')
        rate = rospy.Rate(30)  # 30hz

        while not rospy.is_shutdown():

            if self.curr_pose is not None and self.curr_twist is not None:

                odom_msg = Odometry()
                odom_msg.header = Header()
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = self.rover_name
                odom_msg.pose = PoseWithCovariance()
                odom_msg.pose.pose = self.curr_pose
                odom_msg.twist = TwistWithCovariance()
                odom_msg.twist.twist = self.curr_twist

                self.gt_pub.publish(odom_msg)

            if self.init_pose is not None and self.init_twist is not None:
                self.init_pose_pub.publish(self.init_pose)

            rate.sleep()

    def model_states_callback(self, data):

        for i in range(len(data.name)):

            if data.name[i] == self.rover_name:

                # if self.set_init_pose < 200:
                #
                #     if self.set_init_pose == 0:
                #         rospy.loginfo("[gt_node] Setting initial rover position...")
                #         rospy.loginfo(data.pose[i])
                #     elif self.set_init_pose == 200:
                #         rospy.loginfo("[gt_node] Initial rover position has been set.")
                #         rospy.loginfo(data.pose[i])
                #
                #     self.init_pose = data.pose[i]
                #     self.init_twist = data.twist[i]
                #     self.set_init_pose = self.set_init_pose + 1
                # else:
                #     self.curr_pose = data.pose[i]
                #     self.curr_twist = data.twist[i]

                self.curr_pose = data.pose[i]
                self.curr_twist = data.twist[i]

if __name__ == "__main__":
    RoverLocalizationGT()
