#!/usr/bin/env python

# Algorthim to move the Rover's arm
# Utilizes Nasa's exacvator message type
# 
# Maintainer: Sam Silverman

import rospy
import thread
import math
import tf
import os
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rosnode
from srcp2_msgs.msg import *
from std_msgs.msg import Float64



class ExacvatorArmController():
    def __init__(self):
     
        rospy.init_node("arm")

        self.bucket_mass = 0
        self.bucket = rospy.Subscriber('/excavator_1/bucket_info', ExcavatorMsg, self.bucket_callback)

        # Arm publishers
        self.move_mount_joint = rospy.Publisher('/excavator_1/mount_joint_controller/command', Float64, queue_size=1 )
        self.move_basearm_joint = rospy.Publisher('/excavator_1/basearm_joint_controller/command', Float64, queue_size=1 )
        self.move_distalarm_joint = rospy.Publisher('/excavator_1/distalarm_joint_controller/command', Float64, queue_size=1 )
        self.move_bucket_joint = rospy.Publisher('/excavator_1/bucket_joint_controller/command', Float64, queue_size=1 )

    def bucket_callback(self, msg):
        self.bucket_mass = msg.mass_in_bucket 
        rospy.loginfo(self.bucket_mass)

    def check_bucket(self):
        rate = rospy.Rate(1)
        while True:
            rospy.loginfo("Current Bucket Mass")
            rospy.loginfo(self.bucket_mass)
            rate.sleep()

    def mount_mover(self):
        rospy.loginfo("Do we get into the function?")
        rate = rospy.Rate(0.3)

        basearm_angle = Float64(math.pi/5)
        distalarm_angle = Float64(math.pi/3)
        bucket_angle = Float64(1.25 * math.pi)
        distalarm_angle_2 = Float64(0)
        basearm_angle_2 = Float64(0)
        mount_angle = Float64(math.pi)
        mount_angle_2 = Float64(0)


        while True:

            self.move_basearm_joint.publish(basearm_angle)
            rate.sleep()
            self.move_distalarm_joint.publish(distalarm_angle)
            rate.sleep()
            self.move_bucket_joint.publish(bucket_angle)
            rate.sleep()
            self.move_distalarm_joint.publish(distalarm_angle_2)
            rate.sleep()
            self.move_basearm_joint.publish(basearm_angle_2)
            rate.sleep()
            self.move_mount_joint.publish(mount_angle)
            rate.sleep()
            self.move_mount_joint.publish(mount_angle_2)
            rate.sleep()

    



def main():
    try:
        mover = ExacvatorArmController()
        mover.mount_mover()
        rospy.spin()

    except:
        rospy.loginfo('Exacvator Arm Mover node terminated')

if __name__ == "__main__":
    main()