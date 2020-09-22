#!/usr/bin/env python


"""
Author : Jonathan Sanabria
Created : 6-14-2020


"""

import rospy
import numpy as np

from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped , Quaternion
from tf.transformations import quaternion_from_euler , euler_from_quaternion
"""
TO RUN FOR TESTING :

source devel/setup.bash
rosrun csi_rover_obstacle_avoidance service_tester.py 

To turn off the service use the client_name == 'off'

rosservice call /scout_1/target_coordinate "x: 0.0
y: -10.0
client_name: 'off'" 

"""
class GoalPublisher():

  def __init__(self):
    rospy.init_node('goal_publisher')
    self.curr_location = None
    self.heading = None


    self.RATE = 30
    rate = rospy.Rate(self.RATE)
    rospy.loginfo('Initing goal publisher')
    rospy.Subscriber("/debugging/gt_odom",Odometry , callback=self.odom_cb )
    #rospy.Subscriber("/" + self.rover_name + "/ekf_odom",Odometry , callback=self.odom_cb )
    #rospy.Subscriber("/" + self.rover_name + "/wheel_odom",Odometry , callback=self.odom_cb )
        # coordinate Queue example
    self.coord_deque = deque([(5,0),(13,0),(17,0),(25,0),(30,0),(35,0),(40,0),(45,0),(50,0),(55,0),(60,0),(65,0),(70,0),(75,0),(80,0),(85,0),(90,0),(95,0),(95,1),(100,0)])
    #deque([(5,0),(13,-2),(17,-2),(25,0),(30,4),(35,4),(35,0),(35,1),(36,1)])
    #deque([(6,0),(13,-2),(15,-2),(20,0),(25,4),(30,4),(35,0),(35,1),(36,1)])
    #deque([(6,0),(13,-4),(15,0),(23,-6),(23,-20),(30,-20),(30,20)])
    #deque([(6,-15),(13,0),(15,2),(20,-16),(23,-20),(30,-20),(30,20)])**********
    #deque([(5,0),(10,4),(15,0),(20,-2),(20,20),(30,20),(30,-20)])
    """deque([(-5,-2),(-5,-12),(-10,0),(13,15),
    (20,20),(0,0),(10,-10),(13,-15),
    (20,-20),(-10,10),(-13,15),(-20,20),
    (0,0),(-10,-10),(-13,-15),(-20,-20)])"""

    self.goal_pose = PoseStamped()
    self.update_goal_pose() #
    self.sent_goal = False 

    self.rover_name = rospy.get_param('rover_name', "scout_1")
    self.goal_topic = "/move_base_simple/goal"
    self.goal_pub = rospy.Publisher( self.goal_topic, PoseStamped, queue_size=1)   

    while not rospy.is_shutdown():

      rospy.loginfo_throttle(2, "current_target : " + str( self.target ) )
      if( not self.sent_goal ):
        self.goal_pub.publish( self.goal_pose )
        self.sent_goal = True
      rate.sleep()

  def update_goal_pose(self):
    self.target = np.asarray( self.coord_deque.popleft() )

    self.goal_pose = PoseStamped()
    self.goal_pose.header.frame_id = "scout_1_tf/chassis"
    self.goal_pose.header.stamp = rospy.Time.now()

    self.goal_pose.pose.position.x = self.target[0]
    self.goal_pose.pose.position.y = self.target[1]
    self.goal_pose.pose.position.z = 0.0

    q = quaternion_from_euler(0.0, 0.0, np.deg2rad(0.0))
    self.goal_pose.pose.orientation = Quaternion(*q)

  # ======================================
  # ========== ODOMETRY       ============
  # ======================================
  #  
  def odom_cb( self , data ):
    self.curr_location = np.array([ data.pose.pose.position.x, data.pose.pose.position.y ])
    (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x,
                          data.pose.pose.orientation.y,
                          data.pose.pose.orientation.z,
                          data.pose.pose.orientation.w] )
    self.heading = yaw

    # without any delay 
    # check if arrived at target
    rounded_result = np.around(self.curr_location,0) == np.around(self.target,0)
    if( rounded_result.all() ):
      update_goal_pose()
      self.sent_goal = False
      rospy.loginfo_throttle(2, "Yay rover arrived !" )



if __name__ == '__main__':
    try:     
        GoalPublisher()
    except Exception as e:
        rospy.loginfo('goal_publisher node terminated')
        rospy.loginfo(e)

