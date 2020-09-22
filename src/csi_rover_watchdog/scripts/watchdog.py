#!/usr/bin/env python
# Watch Dog Node.
#
# This program will watch for anomolies in the Rover/Simulation behavior,
# and reset the Rover/throw error messages if necessary
#
# Maintainers: Mikey (Chris) Calloway, Neil Nie
# Contributors: Mikey (Chris) Calloway, Neil Nie , Jonathan Sanabria


import rospy
import thread
import tf
import os
import numpy as np
from geometry_msgs.msg import Twist, Point, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from srcp2_msgs.msg import *
from srcp2_msgs.srv import ResetModelSrvRequest, ResetModelSrvResponse, ResetModelSrv
from rtabmap_ros.srv import ResetPose
import rosnode


class WatchDog():
    def __init__(self):

        rospy.init_node( "watchdog" )

        self.rover_name = rospy.get_param('rover_name', "scout_1")
        
        self.odom_dict = { "ekf":{"pose":PoseWithCovariance(),"roll":0,"pitch":0,"yaw":0},
                            "visual":{"pose":PoseWithCovariance(),"roll":0,"pitch":0,"yaw":0},
                            "wheel" :{"pose":PoseWithCovariance(),"roll":0,"pitch":0,"yaw":0}}
        rospy.loginfo("Setting watchdog subscribers")

        ################
        #   subscribers
        ################
        self.scan_sub = rospy.Subscriber('/scout_1/laser/filtered', LaserScan, self.scan_callback)
        self.ekf_odometry = rospy.Subscriber("/"+self.rover_name+'/ekf_odom', Odometry, self.ekf_odometry_callback)
        self.vis_odometry = rospy.Subscriber("/rtabmap/visual_odom", Odometry, self.vis_odometry_callback)
        #self.imu_odometry = rospy.Subscriber("/"+self.rover_name+'/imu_odom', Odometry, self.odometry_callback)
        self.whe_odometry = rospy.Subscriber("/"+self.rover_name+'/wheel_odom', Odometry, self.wheel_odometry_callback)
        

        rospy.loginfo("Setting watchdog services")
        ################
        #   services
        ################
        #rosservice call /rtabmap/reset_odom_to_pose "{x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}"
        self.RTAB_reset_service_name = "/rtabmap/reset_odom_to_pose" 
        self.SRCP2_reset_service_name = "/" + self.rover_name + "/reset_model" # Name of the SRCP2 reset service

        self.time_at_last_vis_odom_reset = 0
        self.current_position = Point(0,0,0)

        self.left_scan = float('inf')
        self.right_scan = float('inf')
        self.center_scan = float('inf')

        # Add nodes to check status of here + there "launch" code for restarting
        self.nodes = [ ] # empty because trying respawn="true" in the navigation launches
        # I didn't see the following working VVVV 
        """{
                "node": "/navigation/csi_rover_move_base",
                "cmd": ["rosrun csi_rover_move_base csi_rover_move_base", "rosnode kill /navigation/global_planner"]
            }
        ]"""
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("watchdog node is running")

    def shutdown(self):
        rospy.loginfo('Stopping watch dog algorithm...')
        rospy.sleep(1)



    def ekf_odometry_callback(self, msg ):
        odom_key = "ekf"
        # Get orientation of rover, and conver it from quaternion to euler 
        rotation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(rotation)

        self.odom_dict[ odom_key ]["pose"] = msg.pose
        self.odom_dict[ odom_key ]["roll"]  = euler[0]
        self.odom_dict[ odom_key ]["pitch"] = euler[1]
        self.odom_dict[ odom_key ]["yaw"]   = euler[2]

        # may want to check values < min and > max depending on performance
        self.current_position = msg.pose.pose.position

    def vis_odometry_callback(self, msg ):
        odom_key = "visual"
        # Get orientation of rover, and conver it from quaternion to euler 
        rotation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(rotation)

        self.odom_dict[ odom_key ]["pose"] = msg.pose
        self.odom_dict[ odom_key ]["roll"]  = euler[0]
        self.odom_dict[ odom_key ]["pitch"] = euler[1]
        self.odom_dict[ odom_key ]["yaw"]   = euler[2]
        self.check_and_correct_visual_odom( msg.header.stamp.secs )

    def wheel_odometry_callback(self, msg ):
        odom_key = "wheel"
        # Get orientation of rover, and conver it from quaternion to euler 
        rotation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(rotation)

        self.odom_dict[ odom_key ]["pose"] = msg.pose
        self.odom_dict[ odom_key ]["roll"]  = euler[0]
        self.odom_dict[ odom_key ]["pitch"] = euler[1]
        self.odom_dict[ odom_key ]["yaw"]   = euler[2]

    def scan_callback(self, msg):
       
        self.left_scan = msg.ranges[-1]
        self.right_scan = msg.ranges[0]
        self.center_scan = msg.ranges[len(msg.ranges) / 2]

    def ping_node(self, node_name, cmd):
        rate = rospy.Rate(1) 
        ping_success = rosnode.rosnode_ping(node_name, 2)
        rate.sleep()

        # cmd_str = "This is the cmd: " + str(cmd[0])
        # rospy.logwarn(cmd_str)

        if (ping_success == False):
            rospy.logerr("" + node_name + " is not runinng.")
            rospy.logerr("Relaunching "  + node_name + " now...")
            for i in cmd:
                thread.start_new_thread(os.system, (i,)) # Make sure to keep comma. Must be a tuple for thread library to work

    # TODO : compare visual to ekf odometry, if not similar by ## meters 
    #                                          or yaw not similar by # radians
    #                                              then determine which is correct....
    #       last_update = time_in_sec
    def check_and_correct_visual_odom(self , time_in_sec ):
        # if covariance more than 10 the odom is PROBABLY jumping
        COVARIANCE_LIMIT = 10

        covariance_result = any( COVARIANCE_LIMIT < covar for covar in self.odom_dict[ "visual" ]["pose"].covariance )
        if( covariance_result 
            and (self.time_at_last_vis_odom_reset + 1 ) < time_in_sec ):
            self.time_at_last_vis_odom_reset = time_in_sec
            reset_service = rospy.ServiceProxy( self.RTAB_reset_service_name , ResetPose )
            ekf_dict = self.odom_dict[ "ekf" ]
            response = reset_service(  x = ekf_dict["pose"].pose.position.x ,
                                        y = ekf_dict["pose"].pose.position.y,
                                        z = ekf_dict["pose"].pose.position.z,
                                        roll  = ekf_dict["roll"],
                                        pitch = ekf_dict["pitch"],
                                        yaw   = ekf_dict["yaw"] )
            if( response ):
                rospy.logerr("IMPORTANT: rover visual odom reset")


    # Checks if rover has flipped over
    def check_rover_stuck_upside_down(self):
        condition = np.absolute( self.odom_dict["ekf"]["roll"] -3 )
        # str_condition = "This is condition: " + str(condition)
        # rospy.loginfo(str_condition )
        if (condition < 0.3):
            rospy.logerr("CRITICAL FAILURE: Rover is on its back")
            reset_service = rospy.ServiceProxy( self.SRCP2_reset_service_name , ResetModelSrv)
            response = reset_service( reset = True )
            if( response.finished ):
                rospy.logerr("IMPORTANT: Rover has been reset")

    # Checks if rover is stuck on a rock or other obstacle
    def check_rover_stuck_forward(self):
        if (self.center_scan < 2):
            rospy.logwarn("Checking if Rover is stuck")
            wait_rate = rospy.Rate(0.01)
            saved_position = self.current_position
            wait_rate.sleep()

            delta_x = np.absolute(saved_position.x - self.current_position.x)
            delta_y = np.absolute(saved_position.y - self.current_position.y)

            # 3 pieces of evidence that the rover is stuck after 10 seconds
            if(self.center_scan < 2 and delta_x < 0.5 and delta_y <0.5 ):
                rospy.logerr("CRITICAL FAILURE: Rover is stuck")
                reset_service = rospy.ServiceProxy( self.SRCP2_reset_service_name , ResetModelSrv)
                response = reset_service( reset = True )
                if( response.finished ):
                    rospy.logerr("IMPORTANT: Rover has been reset")


    # Checks if a rover is stuck in the same position for a while
    def check_rover_stuck_position(self):
        STUCK_DISTANCE = 9.9e-05

        prev_position = self.current_position
        rate.sleep()
        delta_distance = np.sqrt(pow(prev_position.x - self.current_position.x,2)+
                                pow(prev_position.y - self.current_position.y,2)) 
        delta_x = np.absolute(prev_position.x - self.current_position.x)
        delta_y = np.absolute(prev_position.y - self.current_position.y)

        if( delta_distance < STUCK_DISTANCE):
            rospy.logwarn("Checking if Rover is stuck")

            rate_long.sleep()
            delta_x = np.absolute(prev_position.x - self.current_position.x)
            delta_y = np.absolute(prev_position.y - self.current_position.y)

            if(delta_x < 0.3 and delta_y < 0.3):
                rospy.logerr("CRITICAL FAILURE: Rover is stuck")
                reset_service = rospy.ServiceProxy( self.SRCP2_reset_service_name , ResetModelSrv)
                response = reset_service( reset = True )
                if( response.finished ):
                    rospy.logerr("IMPORTANT: Rover has been reset")


    # This function watches over the system, 
    # and will throw errors and call resets where necessary
    def watchdog(self):
        # wait 5 sec for other code to finish launching
        rospy.Rate(.3).sleep() 
        
        rate_long = rospy.Rate(0.05)
        rate = rospy.Rate(0.3) 
        while not rospy.is_shutdown():
            rospy.loginfo("TESTING IF WATCHDOG NODE IS WORKING")


            # Ping nodes.
            # If something is doesn't respond, we restart the node
            for i in self.nodes:
                rospy.loginfo(i)
                self.ping_node(i["node"], i["cmd"])
                rate.sleep()
          
            # Checks if Rover is on its back.
            # If it is, then a reset call is sent
            self.check_rover_stuck_upside_down()
               
            # Checks of Rover is stuck forward facing
            # Gives rover benefit of the doubt for 10 seconds
            # before doing a reset call
            # self.check_rover_stuck_forward()
                        
            # Check if rover is in the same spot for 20 seconds
            # self.check_rover_stuck_position()
   
            rate.sleep()

def main():
    try:
        watch = WatchDog()
        rospy.Rate(1).sleep()
        watch.watchdog()
        rospy.spin()
    except Exception as e:
        rospy.loginfo('watchdog node terminated : ' + str(e))

if __name__ == "__main__":
    main()
