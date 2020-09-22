#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class SimpleRoverController:

    def __init__(self):

        self.namespace = rospy.get_param("name_space", "scout_1")
        self.w_s = rospy.get_param("wheel_separation", 1.7680)                  # wheel seperation
        self.w_r = rospy.get_param("wheel_separation", 0.3048)                  # wheel radisu

        if "/" in self.namespace:
            rospy.logerr("[rover_motion_controller] invalid namespace. namespace can not contain /")
            exit(1)

        self.lf_steering_pub = rospy.Publisher("/" + self.namespace + "/fl_steering_arm_controller/command", Float64, queue_size=2)
        self.rf_steering_pub = rospy.Publisher("/" + self.namespace + "/fr_steering_arm_controller/command", Float64, queue_size=2)
        self.lr_steering_pub = rospy.Publisher("/" + self.namespace + "/bl_steering_arm_controller/command", Float64, queue_size=2)
        self.rr_steering_pub = rospy.Publisher("/" + self.namespace + "/br_steering_arm_controller/command", Float64, queue_size=2)

        self.lf_axle_pub = rospy.Publisher("/" + self.namespace + "/fl_wheel_controller/command", Float64, queue_size=2)
        self.rf_axle_pub = rospy.Publisher("/" + self.namespace + "/fr_wheel_controller/command", Float64, queue_size=2)
        self.lr_axle_pub = rospy.Publisher("/" + self.namespace + "/bl_wheel_controller/command", Float64, queue_size=2)
        self.rr_axle_pub = rospy.Publisher("/" + self.namespace + "/br_wheel_controller/command", Float64, queue_size=2)

        self.steering_cmd = 0
        self.linear_vel = 0
        self.linear_x = 0
        self.angular_z = 0

        rospy.Subscriber("/csi_rover/cmd_vel", Twist, callback=self.directional_movement)

        rospy.init_node('rover_motion_controller', anonymous=True)
        rate = rospy.Rate(30)  # 10hz

        while not rospy.is_shutdown():

            # check to see if there's an explicit yaw command
            if self.angular_z != 0:
                self.rf_axle_pub.publish((self.linear_x + self.angular_z * self.w_s / 2.0) / self.w_r)
                self.rr_axle_pub.publish((self.linear_x + self.angular_z * self.w_s / 2.0) / self.w_r)
                self.lf_axle_pub.publish((self.linear_x - self.angular_z * self.w_s / 2.0) / self.w_r)
                self.lr_axle_pub.publish((self.linear_x - self.angular_z * self.w_s / 2.0) / self.w_r)

                # lock all steering joints to be zero
                self.synchronized_steering(0)

            # else use crab steering
            else:
                self.lf_axle_pub.publish(self.linear_vel)
                self.lr_axle_pub.publish(self.linear_vel)
                self.rf_axle_pub.publish(self.linear_vel)
                self.rr_axle_pub.publish(self.linear_vel)

                self.synchronized_steering(self.steering_cmd)

            rate.sleep()

    # move all of the steering joints to a position.
    # the parameter is an angle value in radians
    def synchronized_steering(self, angle):

        self.lf_steering_pub.publish(angle)
        self.rf_steering_pub.publish(angle)
        self.lr_steering_pub.publish(angle)
        self.rr_steering_pub.publish(angle)

    # Determine steering angle
    # Set linear_vel as magnitude
    # Range -pi/2 to pi/2
    # else use skid_steering
    def directional_movement(self, data):
        # data comes in as ( x , y )
        # https://answers.ros.org/question/29706/twist-message-example-and-cmd_vel/
        # rospy.loginfo("Received a /cmd_vel message!")
        # rospy.loginfo("Linear Components: [%f, %f, %f]"%(data.linear.x, data.linear.y, data.linear.z))
        # rospy.loginfo("Angular Components: [%f, %f, %f]"%(data.angular.x, data.angular.y, data.angular.z))

        theta = math.atan2(data.linear.x, data.linear.y)
        self.steering_cmd = theta
        self.linear_vel = math.sqrt(math.pow(data.linear.x, 2) + math.pow(data.linear.y, 2))

        self.angular_z = data.angular.z
        self.linear_x = data.linear.x


if __name__ == '__main__':
    try:
        SimpleRoverController()
    except rospy.ROSInterruptExoception:
        pass
