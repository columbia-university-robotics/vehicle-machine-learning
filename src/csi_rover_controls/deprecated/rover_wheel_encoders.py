#!/usr/bin/env python
"""
This script publishes the joint encoder values and the linear velocities of
each of the wheels.

The script subscribes to /scout_1/joint_states topic. This topic is
published at 15 hz, which is the bottleneck of the publishers in this
class.
"""

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class RoverWheelEncoders:

    def __init__(self):

        # user can change the naming of the axles
        self.namespace = rospy.get_param("namespace", "scout_1")
        self.fl_axle_name = rospy.get_param("fl_wheel_joint_name", "fl_wheel_joint")
        self.fr_axle_name = rospy.get_param("fr_wheel_joint_name", "fr_wheel_joint")
        self.bl_axle_name = rospy.get_param("bl_wheel_joint_name", "bl_wheel_joint")
        self.br_axle_name = rospy.get_param("br_wheel_joint_name", "br_wheel_joint")
        self.w_r = rospy.get_param("wheel_radius", 0.275)

        if "/" in self.namespace:
            rospy.logerr("[rover_motion_controller] invalid namespace. namespace can not contain /")
            exit(1)

        # publishers
        fr_wheel_pub = rospy.Publisher("/" + self.namespace + "/fr_wheel/encoder_value", Float64, queue_size=5)
        fl_wheel_pub = rospy.Publisher("/" + self.namespace + "/fl_wheel/encoder_value", Float64, queue_size=5)
        br_wheel_pub = rospy.Publisher("/" + self.namespace + "/br_wheel/encoder_value", Float64, queue_size=5)
        bl_wheel_pub = rospy.Publisher("/" + self.namespace + "/bl_wheel/encoder_value", Float64, queue_size=5)

        fr_wheel_vel_pub = rospy.Publisher("/" + self.namespace + "/fr_wheel/linear_vel", Float64, queue_size=5)
        fl_wheel_vel_pub = rospy.Publisher("/" + self.namespace + "/fl_wheel/linear_vel", Float64, queue_size=5)
        br_wheel_vel_pub = rospy.Publisher("/" + self.namespace + "/br_wheel/linear_vel", Float64, queue_size=5)
        bl_wheel_vel_pub = rospy.Publisher("/" + self.namespace + "/bl_wheel/linear_vel", Float64, queue_size=5)

        # create and initialize the class variables.
        self.fr_wheel_pos = 0
        self.fl_wheel_pos = 0
        self.br_wheel_pos = 0
        self.bl_wheel_pos = 0

        self.fr_wheel_vel = 0
        self.fl_wheel_vel = 0
        self.br_wheel_vel = 0
        self.bl_wheel_vel = 0

        # subscribe to the joint state topic.
        rospy.Subscriber("/" + self.namespace + "/joint_states", JointState, callback=self.joint_states_callback)

        rospy.init_node('rover_wheel_encoders', anonymous=True)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            fr_wheel_pub.publish(self.fr_wheel_pos)
            fl_wheel_pub.publish(self.fl_wheel_pos)
            br_wheel_pub.publish(self.br_wheel_pos)
            bl_wheel_pub.publish(self.bl_wheel_pos)

            # the wheel velocities are not the same as the angular velocities of the joints.
            # the wheel velocities are linear (tangential) velocities of the each wheel
            fr_wheel_vel_pub.publish(self.fr_wheel_vel)
            fl_wheel_vel_pub.publish(self.fl_wheel_vel)
            br_wheel_vel_pub.publish(self.br_wheel_vel)
            bl_wheel_vel_pub.publish(self.bl_wheel_vel)

            rate.sleep()

    def joint_states_callback(self, msg):

        names = msg.name
        position = msg.position

        for i in range(0, len(names)):

            # pick out all of the axle position and
            # assign them to the class variables.

            if names[i] == self.fl_axle_name:
                self.fl_wheel_vel = msg.velocity[i] * self.w_r
                self.fl_wheel_pos = position[i]
            elif names[i] == self.bl_axle_name:
                self.bl_wheel_vel = msg.velocity[i] * self.w_r
                self.bl_wheel_pos = position[i]
            elif names[i] == self.fr_axle_name:
                self.fr_wheel_vel = msg.velocity[i] * self.w_r
                self.fr_wheel_pos = position[i]
            elif names[i] == self.br_axle_name:
                self.br_wheel_vel = msg.velocity[i] * self.w_r
                self.br_wheel_pos = position[i]


if __name__ == "__main__":
    try:
        RoverWheelEncoders()
    except rospy.ROSInterruptException:
        pass
