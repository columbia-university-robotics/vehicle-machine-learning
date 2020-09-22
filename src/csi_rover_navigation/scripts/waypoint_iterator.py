#!/usr/bin/env python

from math import *

"""
Issues :
Throws error that -----
" None of the points of the global plan were in the local costmap,
 global plan points too far from robot "

When it is spawned at the location. Does not automatically return with client.wait_for_result()
-- should probably try some logic in the move_base class.

referenced :
-http://wiki.ros.org/actionlib
-http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient

- https://answers.ros.org/question/330776/how-to-send-a-new-goal-movebasegoal-without-interrupting-the-node-python/
- https://github.com/MobileRobots/ros-arnl/blob/master/goal_action_client_example.py
"""
import rospy
import actionlib
from std_msgs.msg import Header
from nav_msgs.msg import Odometry , Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionFeedback , MoveBaseActionGoal , MoveBaseActionResult , MoveBaseGoal , MoveBaseAction

class WayPointIterator():

    def __init__(self , p ):

        self.path = p
        self.idx = 0
        self.rover_name = rospy.get_param('/rover_name')
        self.path_sub = rospy.Subscriber("/navigation/global_planner/path", Path, queue_size=10)
        self.odom_sub = rospy.Subscriber( "/" + self.rover_name + "/ekf_odom",
                                          Odometry , self.update_current_location )
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.initial_pose_set = False

        while( not self.initial_pose_set ):
            rospy.loginfo_throttle(2, "Waiting for Initial Position parameter.")

        self.client = actionlib.SimpleActionClient('csi_move_base', MoveBaseAction )
        self.client.wait_for_server()
        self.set_new_target()

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            self.move_base_action = self.get_action()
            rospy.loginfo_throttle(2, "Sending Goal ..."+str(self.move_base_action.action_goal.goal))
            self.client.send_goal( self.move_base_action.action_goal.goal )
            print( 'Result received. Action state is %s' % self.client.get_state())
            print( 'Goal status message is %s' % self.client.get_goal_status_text())

            print(self.client.get_result() )
            #rospy.loginfo_throttle(2, "Wait for result ...")
            #result = self.client.wait_for_result()
            while( not self.arrived() ):
              rospy.loginfo_throttle(1, "Action state is %s" % self.client.get_state() )
            rate.sleep()

    # check if rover has arrived at target location
    def arrived(self):
        if( round(self.curr_x,0)!=round(self.target_x,0)
         or round(self.curr_y,0)!=round(self.target_y,0)):
            return False
        self.set_new_target()
        return True

    def set_new_target(self):
      if( self.client.get_state() ):
          self.client.cancel_goal()
      self.target_waypoint = self.path.poses[ self.idx ]
      self.target_x = self.target_waypoint.pose.position.x
      self.target_y = self.target_waypoint.pose.position.y
      self.idx += 1

    def update_current_location( self , data ):
        rospy.loginfo_throttle(2, "update_current_loc parameter.")
        if( not self.initial_pose_set ):
            self.initial_pose_set = True
        self.curr_x = data.pose.pose.position.x
        self.curr_y = data.pose.pose.position.y

    def publish_goal( self ):
        #rospy.loginfo_throttle(2, "X : "+str(round(self.target_x,1))+" Y: "+str(round(self.target_y,1)))
        rospy.loginfo_throttle(2, "X : "+str(self.target_y) )
        self.goal_pub.publish( self.target_waypoint )

    def goal_client(self):
        goal = self.target_waypoint
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def get_action(self):
        mba = MoveBaseAction()
        ag_ = MoveBaseActionGoal()
        ag_.header = Header()
        ag_.header.frame_id = "map"
        ag_.header.stamp = rospy.Time.now()
        ag_.goal = MoveBaseGoal()
        ag_.goal.target_pose = self.target_waypoint
        mba.action_goal = ag_
        return mba

if __name__ == '__main__':
    try:     
        WayPointIterator()
    except e:
        rospy.loginfo(e)

