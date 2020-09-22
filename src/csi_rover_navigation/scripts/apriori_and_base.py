#!/usr/bin/env python

from math import *
import numpy as np  # for dot products and normals

import rospy
from std_msgs.msg import Bool , Header
from tf.transformations import quaternion_from_euler , euler_from_quaternion
from nav_msgs.msg import Odometry , Path
from srcp2_msgs.srv import AprioriLocationSrv, HomeLocationSrv, HomeAlignedSrv, ToggleLightSrv
from geometry_msgs.msg import PoseStamped , PointStamped , Point , Twist

"""
TO RUN FOR TESTING :

source devel/setup.bash
rosrun csi_rover_navigation apriori_and_base.py
 
"""
class APrioriAndBase():

    def __init__(self):
        rospy.init_node('APrioriAndBase_node')
        self.RATE = 30
        self.rate = rospy.Rate(self.RATE)
        self.initial_pose_set = False
        self.debug = False

        self.rover_name = rospy.get_param('/rover_name')
        self.odom = rospy.Subscriber( "/"+self.rover_name+"/ekf_odom", Odometry, self.update_current_loc )
        self.cube_sub = rospy.Subscriber( "/cube_position", PointStamped, self.cube_cb)
        self.base_sub = rospy.Subscriber( "/base_station/pose", PoseStamped, self.base_cb)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher("/navigation/global_planner/path", Path, queue_size=10)
        self.new_plan_pub = rospy.Publisher("/"+self.rover_name+"/new_plan", Bool, queue_size=100)
        self.cmd_vel_pub = rospy.Publisher( "/"+self.rover_name+"/cmd_vel", Twist, queue_size=10)

        self.global_cmd_vel = Twist()
        # toggle light
        try:
            service = rospy.ServiceProxy( '/'+self.rover_name+'/toggle_light' , ToggleLightSrv )
            is_true = service( 'high' )
        except rospy.ServiceException, e:
            rospy.logerr( "Service call failed: %s"%e )
            self.report_location( point_msg , service_name , service_type )


        # check whether rover knows it's own location
        while( not self.initial_pose_set ):
            rospy.loginfo_throttle(2, "Waiting for Initial Position parameter.")
            self.rate.sleep()

        self.cube_loc = None
        self.base_loc = None
        
        self.cube_detected = False
        self.base_detected = False
        # locate a priori object
        self.locate_cube() # by rotating in place
        self.report_location( self.cube_loc , "/apriori_location_service" , AprioriLocationSrv )
        # locate base
        self.locate_base() # by rotating in place
        self.send_move_base_path() 
        while( not self.rover_at_base() ):
            self.rate.sleep()
        self.call_home_service("/arrived_home_service", HomeLocationSrv)         
        self.new_plan_pub.publish(True)   
                    
        # if vision SEES a priori location
        #       "The a priori location service can be called with the Scout in any location."
        #   then call rosservice call /apriori_location_service "pose:
        #                x: 66.0
        #                y: -34.0
        #                z: 17.0" 
        #              result: True
        # a priori obj is bounded on Z axis from -- 5 to 25 meters on the z-axis
        
        # /arrived_home_service             srcp2_msgs/HomeLocationSrv
        # /aligned_service                  srcp2_msgs/HomeAlignedSrv


    def send_move_base_path(self):
        p = self.make_path_from_base_loc()
        self.path_pub.publish(p)


    def make_path_from_base_loc(self):
        NUM_OF_BASE_LOC_COPIES = 10
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.poses = []
        self.base_loc.header.frame_id = "map"

        for count in range(NUM_OF_BASE_LOC_COPIES):
            #pose = PoseStamped()
            #pose.header = Header()
            self.base_loc.header.seq = count
            self.base_loc.pose.orientation = self.curr_orientation

            path.poses.append(self.base_loc)
            count += 1
        return path

    def make_path_from_new_vantage(self):
        NUM_OF_BASE_LOC_COPIES = 10
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.poses = []

        for count in range(NUM_OF_BASE_LOC_COPIES):
            pose = PoseStamped()
            pose.header = Header()
            pose.header.seq = count
            pose.header.frame_id = "map"
            pose.pose.position.x = self.vantage_point_loc_x
            pose.pose.position.y = self.vantage_point_loc_y
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
            count += 1
        return path

    def cube_cb(self, msg ):
        if( self.debug ):
            rospy.logerr( "CUBE DETECTED.")
        self.cube_loc = msg.point
        self.cube_detected = True

    def base_cb(self, msg ):
        if( self.debug ):
            rospy.logwarn( "BASE DETECTED.")

        self.base_loc = msg
        self.base_detected = True
              
    def locate_cube(self):
        if( self.debug ):
            rospy.logerr( "LOCATING CUBE.")

        MAX = self.RATE*20 # seconds
        counter = 0
        while( not self.cube_detected ):
            #send explicit steer cmd 
            self.engage_explicit_steer()
            self.rate.sleep()
            counter += 1
            if( MAX < counter ):
                # the rover has spinned for a few seconds 
                # it should spin in another place
                self.set_new_vantage_point()
                p = self.make_path_from_new_vantage()
                self.path_pub.publish(p)
                while( not self.rover_at_new_vantage_point() 
                    and not self.cube_detected):
                    # let move_base take it from now
                    self.rate.sleep()
                if( self.debug ):
                    rospy.logwarn( "self.new_plan_pub.publish(True) .")
                self.new_plan_pub.publish(True) 
                counter = 0 
        self.stop_explicit_steer()

    def locate_base(self):
        if( self.debug ):
            rospy.logerr( "LOCATING BASE.")

        while( not self.base_detected ):
            #send explicit steer cmd 
            self.engage_explicit_steer()
            self.rate.sleep()
     
    def engage_explicit_steer(self):
        self.global_cmd_vel.linear.x = 0.10
        self.global_cmd_vel.angular.x = 1
        self.global_cmd_vel.angular.z = 1
        self.cmd_vel_pub.publish(self.global_cmd_vel)
        

    def stop_explicit_steer(self):
        # is abrupt stop
        self.global_cmd_vel.linear.x = 0
        self.global_cmd_vel.angular.x = 0
        self.global_cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.global_cmd_vel)
        self.new_plan_pub.publish(True)

    def report_location(self, point_msg , service_name , service_type ):
      try:
        service = rospy.ServiceProxy( service_name , service_type )
        is_true = service( point_msg )
        if( not is_true ):
          rospy.logerr( "Cube service returned false.")
        self.cube_detected = False
        self.locate_cube() # by rotating in place
      except rospy.ServiceException, e:
        # When tested in command line NASA seems to be returning 
        # and err when the location is not correct
        # so keep looking beybey
        rospy.logerr( "Service call failed: %s"%e )
        self.cube_detected = False
        self.locate_cube() # by rotating in place

    def call_home_service(self, service_name , service_type ):
      try:
        service = rospy.ServiceProxy( service_name , service_type )
        is_true = service(True)
        if( not is_true ):
          rospy.logerr("Cube service returned false.")
          self.call_home_service( service_name , service_type )
          self.rate.sleep()
      except rospy.ServiceException, e:
        rospy.logerr( "Service call failed: %s"%e )
     

    def set_new_vantage_point(self):
        METER_DISTANCE_FROM_CUR = 5
        q = self.curr_orientation
        r,p,y = euler_from_quaternion( [q.x , q.y, q.z, q.w] )
        self.vantage_point_loc_x = self.curr_x + cos(y)*METER_DISTANCE_FROM_CUR
        self.vantage_point_loc_y = self.curr_y + sin(y)*METER_DISTANCE_FROM_CUR
            
    def update_current_loc( self , data = None ):
        rospy.logdebug( "update_current_loc parameter.")
        if( not self.initial_pose_set ):
            self.initial_pose_set = True
        self.curr_x = data.pose.pose.position.x
        self.curr_y = data.pose.pose.position.y
        self.curr_orientation = data.pose.pose.orientation

    def rover_at_base( self ):
        distance = sqrt( pow(self.curr_x - self.base_loc.pose.position.x ,2) 
                            + pow(self.curr_y - self.base_loc.pose.position.y ,2) )
        return distance < 2 

    def rover_at_new_vantage_point( self ):
        distance = sqrt( pow(self.curr_x - self.vantage_point_loc_x ,2) 
                            + pow(self.curr_y - self.vantage_point_loc_y ,2) )
        if( self.debug ):
            rospy.logerr( "rover_at_new_vantage_point."+str(distance < 2))

        return distance < 2 


    def publish_goal( self ):
        rospy.loginfo_throttle(2, "X : "+str(round(self.target_x,1))+" Y: "+str(round(self.target_y,1)))
        """
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = self.target_x
        p.pose.position.y = self.target_y
        p.pose.orientation = quaternion_from_euler(0,0,0)

        self.goal_pub.publish( p )
        """
        
if __name__ == '__main__':
    try:     
        APrioriAndBase()
    except e:
        rospy.loginfo(e)
