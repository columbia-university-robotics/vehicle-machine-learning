#!/usr/bin/env python

import rospy
import os
import math
from collections import deque
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from srcp2_msgs.srv import LocalizationSrv
from tf import transformations
from gazebo_msgs.msg import ModelStates


def convert_to_euler(orientation):

    quaternion = (orientation.x,
                  orientation.y,
                  orientation.z,
                  orientation.w)

    euler = transformations.euler_from_quaternion(quaternion)
    return euler

class StartupNode:

    def __init__(self):

        rospy.init_node('startup_node', anonymous=True)
        rospy.Subscriber("/scout_1/imu", Imu, callback=self.imu_cb)
        self.imu_data = None
        self.namespace = rospy.get_param("namespace", "localization")
        self.debugging = rospy.get_param("debugging", False)
        self.rn = rospy.get_param("rover_name", "scout_1")

        self.deque = deque(maxlen=60)


        self.lf_steering_pub = rospy.Publisher("/" + self.rn + "/fl_steering_arm_controller/command", Float64,
                                               queue_size=10)
        self.rf_steering_pub = rospy.Publisher("/" + self.rn + "/fr_steering_arm_controller/command", Float64,
                                               queue_size=10)
        self.lr_steering_pub = rospy.Publisher("/" + self.rn + "/bl_steering_arm_controller/command", Float64,
                                               queue_size=10)
        self.rr_steering_pub = rospy.Publisher("/" + self.rn + "/br_steering_arm_controller/command", Float64,
                                               queue_size=10)

        self.lf_axle_pub = rospy.Publisher("/" + self.rn + "/fl_wheel_controller/command", Float64, queue_size=10)
        self.rf_axle_pub = rospy.Publisher("/" + self.rn + "/fr_wheel_controller/command", Float64, queue_size=10)
        self.lr_axle_pub = rospy.Publisher("/" + self.rn + "/bl_wheel_controller/command", Float64, queue_size=10)
        self.rr_axle_pub = rospy.Publisher("/" + self.rn + "/br_wheel_controller/command", Float64, queue_size=10)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            # send zero to all joints
            self.stop_rover_model()

            # wait for first IMU callback
            if self.imu_data is not None:
                if self.ready_to_launch(self.imu_data):
                    rospy.loginfo_throttle(1, "[STN]: Rover Ready to Start...")
                    self.set_initial_param()

            rate.sleep()

    # set all of the joint commands to zero
    def stop_rover_model(self):

        self.lf_axle_pub.publish(0)
        self.rf_axle_pub.publish(0)
        self.rr_axle_pub.publish(0)
        self.lr_axle_pub.publish(0)

        self.lf_steering_pub.publish(0)
        self.rf_steering_pub.publish(0)
        self.lr_steering_pub.publish(0)
        self.rr_steering_pub.publish(0)

    # check to see if the rover is still enough
    # to launch the localization package
    def ready_to_launch(self, imu_data):

        # calculate magnitude of angular velocity
        angular_v = math.sqrt(math.pow(imu_data.angular_velocity.x, 2) +
                              math.pow(imu_data.angular_velocity.y, 2) +
                              math.pow(imu_data.angular_velocity.z, 2))

        # remove z axis from linear acceleration
        euler = convert_to_euler(imu_data.orientation)
        roll = euler[0]
        pitch = euler[1]
        linear_a_x = imu_data.linear_acceleration.x * math.cos(pitch)
        linear_a_y = imu_data.linear_acceleration.y * math.cos(roll)

        # calculate the magnitude of the linear acceleration
        linear_a = math.sqrt(math.pow(linear_a_x, 2) +
                             math.pow(linear_a_y, 2))

        self.deque.appendleft(linear_a + angular_v)
        if len(self.deque) < 50:
            avg = 1.0
        else:
            avg = sum(self.deque) / len(self.deque)

        if avg < 0.9:
            return True
        return False

    # call the ros service to get initial position of rover
    # then set the initial state parameter for the ekf_node
    def set_initial_param(self):

        rospy.wait_for_service("/scout_1/get_true_pose", 2)
        rospy.loginfo("[STN]: Waiting to set param")
        try:
            get_true_pose = rospy.ServiceProxy("/scout_1/get_true_pose", LocalizationSrv)
            response = get_true_pose(True).pose
            euler = convert_to_euler(response.orientation)

            init_pose = [response.position.x, response.position.y, response.position.z,
                         euler[0], euler[1], euler[2],
                         0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0]
            param_name = "/" + self.namespace + "/" + "ekf_odom_node" + "/initial_state"
            rospy.set_param(param_name, init_pose)

            # set rtabmap initial parameter
            pose_str = str(response.position.x) + " " + str(response.position.y) + " " + str(response.position.z) + " "
            pose_str = pose_str + str(euler[0]) + " " + str(euler[1]) + " " + str(euler[2])
            rospy.loginfo(pose_str)
            rospy.set_param("/rtabmap/rgbd_odometry/initial_pose", pose_str)

            # set wheel odom initial parameter
            rospy.set_param("/" + self.namespace + "/rover_wheel_odometry/initial_pose",
                            [response.position.x, response.position.y])

            rospy.loginfo("[STN]: Parameter has been set")
            rospy.loginfo("[STN]: Rover Initial State " + str(init_pose))
            rospy.loginfo("[STN]: Launching Localization Package...")

            rospy.sleep(1)

            rospy.signal_shutdown("Job finished")

        except rospy.ServiceException, e:
            rospy.logerr("true pose service call failed")
            rospy.logerr(str(e))

    def imu_cb(self, data):
        self.imu_data = data



if __name__ == '__main__':
    try:
        StartupNode()
    except rospy.ROSInterruptException:
        pass
