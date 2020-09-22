#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String , Bool
from geometry_msgs.msg import Point
from csi_rover_localizer.srv import TargetCoordinate , TargetCoordinateResponse


class TargetCoordinateService():

	def __init__( self ):

		self.curr_location = None
		self.target_location = None
		self.rover_name = rospy.get_param('rover_name', "scout_1")

		self.odom = rospy.Subscriber( "/" + self.rover_name + "/ekf_odom", Odometry , self.update_current_loc )
		self.srv = rospy.Service( "/" + self.rover_name + "/target_coordinate" , TargetCoordinate , self.target_callback )

		self.off_requested = False
		self.rover_arrived_bool = False

		rospy.loginfo("service is running")


	def current_is_target_location( self  ):
		DP = 0 # Decimal_Precision
		if( self.target_location ):
			if( round(self.curr_location[0], DP )==round(self.target_location[0], DP )
			and round(self.curr_location[1], DP )==round(self.target_location[1], DP )):
				return True
		else : 
			return False

	def target_callback( self , request ):
		# main service logic


		if( request.client_name == "off" ):
			self.off_requested = True
			self.target_location = None
			rospy.loginfo("request.client == "+ request.client_name  )
		elif( request.client_name == "reverse" ):
			pass
		elif( request.client_name == "MAIN_PATH_PLANNER_NAME" ):
			rospy.loginfo( "request.client == " + request.client_name )
		elif( request.client_name == "sub1_PATH_PLANNER_NAME" ):
			rospy.loginfo( "request.client == " + request.client_name )
		elif( request.client_name == "sub2_PATH_PLANNER_NAME" ):
			rospy.loginfo( "request.client == " + request.client_name )
		elif( request.client_name == "sub3_PATH_PLANNER_NAME" ):
			rospy.loginfo( "request.client == " + request.client_name )
		else:#( request.client_name == "" ):
			self.off_requested = False
			self.target_location = [ request.x , request.y ]
			#data=rospy.wait_for_message("/scout_1/ekf_odom", Odometry)

			#rospy.loginfo( "request.client == " + request.client_name + "! is EMPTY" )

		response = TargetCoordinateResponse()
		# update_current_location

		# update_arrived_bool
		if( self.current_is_target_location() ):
			response.arrived = True
		else:
			response.arrived = False

		return response


	def update_current_loc( self , data = None ):
		# self.x_rover = data.pose.pose.position.x
		# self.y_rover = data.pose.pose.position.y
		self.curr_location = [data.pose.pose.position.x, data.pose.pose.position.y]



if __name__ == "__main__" :
	try :
		rospy.init_node( "target_coordinate_service_node" )
		TargetCoordinateService()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
