#!/usr/bin/env python
import rospy
from collections import deque
from csi_rover_localizer.srv import TargetCoordinate 

"""
TO RUN FOR TESTING :

source devel/setup.bash
rosrun csi_rover_obstacle_avoidance service_tester.py 

To turn off the service use the client_name == 'off'

rosservice call /scout_1/target_coordinate "x: 0.0
y: -10.0
client_name: 'off'" 

"""
class ServiceTester():

	def __init__(self):
		rospy.init_node('ServiceClient_example')
		self.RATE = 30
		rate = rospy.Rate(self.RATE)

        # coordinate Queue example
		self.coord_deque = deque([(-5,-2),(-5,-12),(-10,0),(13,15),
         		(20,20),(0,0),(10,-10),(13,-15),
         		(20,-20),(-10,10),(-13,15),(-20,20),
         		(0,0),(-10,-10),(-13,-15),(-20,-20)])

		self.target = self.coord_deque.popleft() 


		# this one is foolproof
		self.rover_name = rospy.get_param('rover_name', "scout_1")
		# this one is better
		#self.rover_name = rospy.get_param('rover_name')
		self.service_name = "/" + self.rover_name + "/target_coordinate"

		rospy.wait_for_service( self.service_name )

		while not rospy.is_shutdown():

			rospy.loginfo_throttle(2, "current_target : " + str( self.target ) )
			# Use the try :... except :
			try:
				target_service = rospy.ServiceProxy( self.service_name , TargetCoordinate)
				response = target_service(  x = self.target[0],
											y = self.target[1],
											client_name = "SCRIPT_NODE_NAME")
											# we can also use Header() but you'd have to input the
											# frame_id so I figured "client_name" was more direct. 
				if( response.arrived ):
					rospy.loginfo_throttle(2, "Yay rover arrived !" )
					self.target = self.coord_deque.popleft() 
			except rospy.ServiceException, e:
				print( "Service call failed: %s"%e )
				rospy.wait_for_service( self.service_name )

			rate.sleep()
			
			

if __name__ == '__main__':
    try:     
        bug1 = Bug1()
        bug1.bug1()
    except:
        rospy.loginfo('bug1 node terminated')
