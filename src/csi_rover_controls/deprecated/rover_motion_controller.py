#!/usr/bin/env python

import sys  # for sys.float_info.min
import time
import math  # for trig functions
import rospy
import numpy as np  # for dot products and normals
# import collections
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry  # for tests


class RoverMotionController:
    AVOID_0_DIVISION = sys.float_info.min  # AVOID_0_DIVISION = 2.2250738585072014e-308

    def __init__(self):

        # self.MAX_SPEED = 1.5
        self.r_name = "/"+rospy.get_param('rover_name')
        self.wheel_separation_length = rospy.set_param(self.r_name+"/wheel_separation_length", 1.5748)
        self.wheel_separation_width = rospy.set_param(self.r_name+"/wheel_separation_width", 1.87325)
        self.wheel_radius = rospy.set_param( self.r_name+"/wheel_radius", 0.275)
        self.wheel_radius = rospy.get_param( self.r_name+"/wheel_radius")
        self.wheel_separation_length = rospy.get_param(self.r_name+"/wheel_separation_length")
        self.wheel_separation_width= rospy.get_param(  self.r_name+"/wheel_separation_width")
        #  INSTANTANEOUS_CENTER_OF_ROTATION_radius
        rospy.set_param( self.r_name+"/icr_radius", 2*1.7680) 
        self.icr_radius = rospy.get_param( self.r_name+"/icr_radius")  
        self.skid_on = rospy.get_param("use_skid_steering", False)

		#------------------------------
		#------- PUBLISHERS -----------
		#..............................
        QUEUE_SIZE = 1

        self.lf_steering_pub = rospy.Publisher( self.r_name + "/fl_steering_arm_controller/command", Float64,
                                               queue_size=QUEUE_SIZE)
        self.rf_steering_pub = rospy.Publisher( self.r_name + "/fr_steering_arm_controller/command", Float64,
                                               queue_size=QUEUE_SIZE)
        self.lr_steering_pub = rospy.Publisher( self.r_name + "/bl_steering_arm_controller/command", Float64,
                                               queue_size=QUEUE_SIZE)
        self.rr_steering_pub = rospy.Publisher(self.r_name + "/br_steering_arm_controller/command", Float64,
                                               queue_size=QUEUE_SIZE)

        self.lf_axle_pub = rospy.Publisher( self.r_name + "/fl_wheel_controller/command", Float64,
                                           queue_size=QUEUE_SIZE)
        self.rf_axle_pub = rospy.Publisher( self.r_name + "/fr_wheel_controller/command", Float64,
                                           queue_size=QUEUE_SIZE)
        self.lr_axle_pub = rospy.Publisher( self.r_name + "/bl_wheel_controller/command", Float64,
                                           queue_size=QUEUE_SIZE)
        self.rr_axle_pub = rospy.Publisher( self.r_name + "/br_wheel_controller/command", Float64,
                                           queue_size=QUEUE_SIZE)

		#------------------------------
		#------- SUBSCRIBERS ----------
		#..............................
        rospy.Subscriber( self.r_name + "/cmd_vel", Twist, callback=self.directional_movement)

        self.yaw_prev = 0

        self.vector_prev = [0, 0]
        self.vector_curr = [-1.123456789, 1.132456789]  # some really random specific number

        self.vector_chassis = np.zeros((1, 2))
        self.vector_pivot_wheel = np.zeros((1, 2))
        self.angle_chassis = 0
        self.angle_pivot_wheel = 0

        self.steer_offset_dict = {"rf": 0, "lf": 0, "lr": 0, "rr": 0}#see four_wheel_steering(...)
        self.radii_offset_dict = {"rf": 0, "lf": 0, "lr": 0,"rr": 0}# need for axle dict, and four_wheel_dif_velo(...)
        self.axles_location_dict = {"rf": (0, 0), "lf": (0, 0), "lr": (0, 0), "rr": (0,0)}  
        self.axles_velocity_dict = {"rf": 0, "lf": 0, "lr": 0, "rr": 0}  
        self.change_to_crab = True
        self.start_wait_time = 0
        self.crab_wait_time = 1.6 # set how long to wait for the steering to match the right angles
        self.explicit_on = False
        # START TIMER TO DELETE AFTER TESTING
        self.start_time = 0
        self.end_time = 0
        self.time_initiated = False


        self.x_rover = 0
        self.y_rover = 0
        self.ONE_ROTATION_TIME = 0
        self.yaw_start = 0
        self.yaw_stop = 0
        # END   TIMER TO DELETE AFTER TESTING

        self.steering_cmd = 0
        self.linear_vel = 0
        self.linear_x = 0
        self.angular_z = 0
        self.lin_x_is_angular_wheel_vel = self.wheel_radius * 2 * math.pi  # self.wheel_radius*2*math.pi  #== 1.72787595947

        rospy.init_node('rover_motion_controller', anonymous=True)
        self.RATE = 30  # used in chassis_alignment(...)
        rate = rospy.Rate(self.RATE)  # self.RATE hz

        while not rospy.is_shutdown():
            # check if skid steering is engaged
            if (self.skid_on):
                #####################################
                #           SKID STEERING           #
                #####################################
                # lock all steering joints to be zero
                self.synchronized_steering(0)

                # SKID STEERING
                self.rf_axle_pub.publish(
                    (self.linear_x + self.angular_z * self.wheel_separation_width / 2.0) / self.wheel_radius)
                self.rr_axle_pub.publish(
                    (self.linear_x + self.angular_z * self.wheel_separation_width / 2.0) / self.wheel_radius)
                self.lf_axle_pub.publish(
                    (self.linear_x + self.angular_z * self.wheel_separation_width / 2.0) / self.wheel_radius)
                self.lr_axle_pub.publish(
                    (self.linear_x + self.angular_z * self.wheel_separation_width / 2.0) / self.wheel_radius)

            # check to see if there's an explicit yaw command
            elif (self.angular_z != 0 ):#and (round(self.angular_z, 3) != round(self.yaw_prev, 3))):
                self.explicit_on = True
                """self.yaw_prev = self.angular_z"""

                #####################################
                #         EXPLICIT STEERING         #
                #####################################
                #####################################
                # STEER ALL WHEELS TO TURN IN PLACE #
                #####################################
                rover_center_to_axle = math.hypot(self.wheel_separation_length / 2,
                                                  self.wheel_separation_width / 2)
                tangent_angle = self.get_tangent_at(self.wheel_separation_length / 2,
                                                    radius=rover_center_to_axle)
                tangent_angle = self.get_theta(np.array([1, 0]), np.array([1, tangent_angle]))
                self.four_wheel_steering(tangent_angle, yaw=True)
                """
                # calculate how many seconds to turn angular_z amount of degrees that many degrees
                CALCULATED_TIME_FOR_ONE_ROTATION = 0
                if (round(self.linear_x, 6) != 0):
                    # time will be infinite if wheel speed is 0
                    CALCULATED_TIME_FOR_ONE_ROTATION = 2 * 7.774999999999864 / (self.linear_x + self.AVOID_0_DIVISION)
                fraction_of_full_rotation = abs(self.angular_z) / (2 * math.pi)
                self.yaw_stop = CALCULATED_TIME_FOR_ONE_ROTATION * fraction_of_full_rotation

                # Below is how rotation time should be calculated
                # but there is something gazebo is doing which is not allowing it to work as such.
                # self.yaw_stop = (math.pi)*(2*(rover_center_to_axle))#/self.lin_x_is_angular_wheel_vel#/((self.lin_x_is_angular_wheel_vel+self.AVOID_0_DIVISION)) )

                #####################################
                # MOVE AXLES FOR "yaw_stop" seconds #
                #####################################
                self.yaw_start = rospy.get_time()
                #while ((rospy.get_time() - self.yaw_start) < self.yaw_stop):
                """
            	self.four_wheel_dif_velo(self.lin_x_is_angular_wheel_vel * self.linear_x, yaw=True)
                """
                #self.four_wheel_dif_velo(0, yaw=True)
                self.yaw_start = 0
                self.yaw_stop = 0
                """
            # reset for a new command of same theta with the key... key == .0101020203210
            elif (self.angular_z == .0000100010001):
                print("in reset ")
                self.yaw_prev = self.angular_z
            # else use crab steering
            elif (self.angular_z == 0):
                self.yaw_prev = self.angular_z
                if( self.explicit_on and not self.change_to_crab ):
                    self.change_to_crab = True
                    self.start_wait_time = rospy.get_time()

                if (not self.chassis_aligned()):
                    self.four_wheel_steering()
                    if( self.change_to_crab and self.crab_wait_time < rospy.get_time() - self.start_wait_time ):
                        self.explicit_on = False
                        self.change_to_crab = False
                        self.four_wheel_dif_velo(self.linear_vel)
                    elif( not self.explicit_on ):
                        self.four_wheel_dif_velo(self.linear_vel)
                else:
                    self.synchronized_steering(self.steering_cmd)
                    if( self.change_to_crab and self.crab_wait_time < rospy.get_time() - self.start_wait_time ):
                        self.explicit_on = False
                        self.change_to_crab = False
                        self.synchronized_axle(self.linear_vel)
                    elif( not self.explicit_on ):
                        self.synchronized_axle(self.linear_vel)
            rate.sleep()

    #######################################
    ##**********************************###
    ##       PUBLISHER   FUNCTIONS      ###
    ##**********************************###
    #######################################

    # move all of the steering joints to a position.
    # the parameter is an angle value in radians
    def synchronized_steering(self, angle):
        print("ss", angle)
        self.lf_steering_pub.publish(angle)
        self.rf_steering_pub.publish(angle)
        self.lr_steering_pub.publish(angle)
        self.rr_steering_pub.publish(angle)

    def synchronized_axle(self, velocity):
        print("sa", velocity)
        self.lf_axle_pub.publish(velocity)
        self.lr_axle_pub.publish(velocity)
        self.rf_axle_pub.publish(velocity)
        self.rr_axle_pub.publish(velocity)

    # move all of the steering joints to a position.
    # the parameter is an angle value in radians
    def four_wheel_steering(self, angle=0, yaw=False):
        if (not yaw):
            print("fws ny")
            self.lf_steering_pub.publish(self.steer_offset_dict["lf"])
            self.rf_steering_pub.publish(self.steer_offset_dict["rf"])
            self.lr_steering_pub.publish(self.steer_offset_dict["lr"])
            self.rr_steering_pub.publish(self.steer_offset_dict["rr"])
        else:
            print("fws  y")
            # Prepare for a turn in place
            self.lf_steering_pub.publish(-angle)
            self.rf_steering_pub.publish(angle)
            self.lr_steering_pub.publish(angle)
            self.rr_steering_pub.publish(-angle)

    # rotate all of the axles (i.e. wheels) .
    def four_wheel_dif_velo(self, pivot_vel, yaw=False):
        if (not yaw):
            print("fwv ny")
            self.lf_axle_pub.publish(self.axles_velocity_dict['lf'])
            self.lr_axle_pub.publish(self.axles_velocity_dict['lr'])
            self.rf_axle_pub.publish(self.axles_velocity_dict['rf'])
            self.rr_axle_pub.publish(self.axles_velocity_dict['rr'])
        else:
            print("fwv  y")
            sign = self.angular_z / abs(self.angular_z + self.AVOID_0_DIVISION)
            self.lf_axle_pub.publish(-pivot_vel*sign )
            self.lr_axle_pub.publish(-pivot_vel*sign )
            self.rf_axle_pub.publish(pivot_vel*sign )
            self.rr_axle_pub.publish(pivot_vel*sign )

    #######################################
    ##**********************************###
    ##       MAIN STEER  FUNCTIONS      ###
    ##**********************************###
    #######################################

    # ======================================
    # ========== MAIN CALL BACK ============
    # ======================================
    # Determine steering angle
    # Set linear_vel as magnitude
    # Range -pi/2 to pi/2    ( should probably be less to account for crab offsets )
    # else use explicit_steering or skid_steering
    def directional_movement(self, data):

        # theta = 0
        self.vector_curr = [round(data.linear.x, 1), round(data.linear.y, 1)]

        # skip setting the self.angular_z and self.linear_x to 0 in skid steering is on
        if not self.skid_on:
            self.angular_z = data.angular.z
            self.linear_x = data.linear.x
            # assign vehicle velocity
            sign = data.linear.x / abs(data.linear.x + self.AVOID_0_DIVISION)
            self.linear_vel = self.linear_x * self.lin_x_is_angular_wheel_vel

        # Once skid steering is engaged we must turn it off if explicit or crab is desired
        if data.angular.x == 1:
            self.skid_on = True

        # if not same then user desires new direction
        if not self.skid_on:
            self.vector_curr = [round(data.linear.x, 1), round(data.linear.y, 1)]
            # if not same then user desires new direction
            if (not self.same_vect(self.vector_prev, self.vector_curr)):
		        self.vector_prev = self.vector_curr
		        hypotenuse = math.hypot(data.linear.x, data.linear.y)
		        if (0 < hypotenuse):
		            # using hypotenuse to see if vector not ( 0 , 0 )
		            theta = math.atan(data.linear.y / (data.linear.x + self.AVOID_0_DIVISION))
		            self.steering_cmd = theta
		            self.angle_chassis = -theta  # will eventually want to have them meet at 0
		            self.angle_pivot_wheel = theta  # will eventually want to have them meet at 0
		        else:
		            # (x,y) = (0,0)
		            self.linear_vel = 0
		            self.steering_cmd = 0

            for key in self.steer_offset_dict.keys():
                self.steer_offset_dict[key] = 0
            for key in self.axles_velocity_dict.keys():
                self.axles_velocity_dict[key] = 0
            self.calc_radii_dictionary()  # for use in four_wheel_dif_velo(...)
            self.calc_axles_dictionary()  # velocity --------------
        elif data.angular.x == -1:
            self.skid_on = False

        # checking whether angular.x ever got turned off
        # had to wait to see if turn off signal was active ( -1 )
        #	 most likely redundant
        if data.angular.x == 0 and self.skid_on:
            self.angular_z = data.angular.z
            self.linear_x = data.linear.x
            # assign vehicle velocity
            self.linear_vel = self.linear_x * self.lin_x_is_angular_wheel_vel


    #######################################
    ##**********************************###
    ##       DICTIONARY  FUNCTIONS      ###
    ##**********************************###
    #######################################

    def calc_axles_dictionary(self):
        # ===============================================
        #       BEGIN GET DIFFERENT AXLE SPEEDS
        # ===============================================
        # With different radii, all velocities will be in relation to the pivot wheel's velocity
        #															current_radius / RADIUS_PIVOT
        # ===============================================
        self.axles_velocity_dict['rf'] = self.linear_vel * (self.radii_offset_dict['rf'] / self.icr_radius)
        self.axles_velocity_dict['lf'] = self.linear_vel * (self.radii_offset_dict['lf'] / self.icr_radius)
        self.axles_velocity_dict['lr'] = self.linear_vel * (self.radii_offset_dict['lr'] / self.icr_radius)
        self.axles_velocity_dict['rr'] = self.linear_vel * (self.radii_offset_dict['rr'] / self.icr_radius)

    # Calculate where an axle is located relative to the pivot wheel being at "(0,0)"
    # The pivot wheel's steering_cmd/angle_pivot_wheel also defines the "x axis"
    def calc_axlelocation_dict(self, pivot_key, adjacent_key, same_side_key, across_key):
        W_SEP_LENGTH = self.wheel_separation_length
        W_SEP_WIDTH = self.wheel_separation_width
        W_SEP_CROSS = math.hypot(W_SEP_LENGTH, W_SEP_WIDTH)

        # wheel postion array       [   sameside   ,   adjacent  ,   across     ]
        wheel_pos_array = np.array([[ W_SEP_LENGTH ,      0      , W_SEP_LENGTH ],
                                    [      0       , W_SEP_WIDTH , W_SEP_WIDTH  ]])
        #calculate whether the axle coord rotation is happening clockwise or counter clockwise
        #
        # it will later determine the slope of the wheel within it's concentric circle around ICR
        # ------- see update_steer_offset for steering use of axles_location_dict
        #
        # it will later determine the radius from the ICR that it's concentric circle makes :
        # ------- see calc_axles_dictionary to see the velocity formula for each axle
        if( 0 < self.angle_pivot_wheel ):
            theta =  -1*self.angle_pivot_wheel
            wheel_pos_array[0,:] *= -1 #update x location
        else:
            theta = abs(self.angle_pivot_wheel)

        self.axles_location_dict[pivot_key] = [0, 0]  # is pivot
        self.axles_location_dict[same_side_key] = self.rotate( wheel_pos_array[:,0] , theta )
        self.axles_location_dict[adjacent_key]  = self.rotate( wheel_pos_array[:,1] , theta )
        self.axles_location_dict[across_key]    = self.rotate( wheel_pos_array[:,2] , theta )

    # Form radius for each circle that the axles are on
    def calc_radii_dictionary(self):
        # ===============================================
        # PRELIMINARY STEPS TO GET DIFFERENT AXLE SPEEDS
        # ===============================================
        pivot_key, adjacent_key, same_side_key, across_key = "", "", "", ""
        if (0 < self.angle_pivot_wheel ):
            # pivot wheel is right front
            pivot_key, adjacent_key = 'rf', 'lf'
            same_side_key, across_key = 'rr', 'lr'
        else:
            # pivot wheel is left front
            pivot_key, adjacent_key = 'lf', 'rf'
            same_side_key, across_key = 'lr', 'rr'
        self.calc_axlelocation_dict(pivot_key, adjacent_key, same_side_key, across_key)

        # ===============================================
        #            NOW READY TO GET RADII
        # ...............................................
        distance_between_pts = lambda pt1, pt2: np.linalg.norm(np.array([pt1[0] - pt2[0],
                                                                         pt1[1] - pt2[1]]))
        for key in self.radii_offset_dict.keys():
            self.radii_offset_dict[key] = distance_between_pts([0, self.icr_radius],
                                                               self.axles_location_dict[key])
        # ===============================================
        #            UPDATE THE STEERING   
        # ...............................................
        INFLECTION_ANGLE = math.pi/2-self.get_theta(np.array([self.wheel_separation_length, 0]),
                          np.array([self.wheel_separation_length, self.wheel_separation_width]))
        _inflection = INFLECTION_ANGLE < abs(self.angle_pivot_wheel)

        self.steer_offset_dict[pivot_key] = self.angle_pivot_wheel
        self.update_steer_offset( self.radii_offset_dict[same_side_key], same_side_key, 
                                       is_sameside = True)
        self.update_steer_offset( self.radii_offset_dict[adjacent_key], adjacent_key ,
                                       is_adjacent = True )
        self.update_steer_offset( self.radii_offset_dict[across_key], across_key,
                                       passed_inflection = _inflection , is_across = True )

    # Create some delta value that corresponds to the difference in angle needed in order to be
    # perpendicular to the line eminating from the Instantaneous Center of Rotation ( ICR )
    def update_steer_offset(self, radius, wheel_name, passed_inflection  = False ,
                                 is_sameside = False , is_adjacent = False , is_across = False ):
        THETA_SIGN = self.angle_pivot_wheel / abs(self.angle_pivot_wheel + self.AVOID_0_DIVISION)
        # ===============================================
        # get the tangent at those points on the circle
        # ...............................................
        tangent_parrallel_to_pt_tan = self.get_tangent_at(self.axles_location_dict[wheel_name][0], abs(radius))

        # ===============================================
        # make vector for wheel according to that tangent_slope
        # ...............................................
        tan_vector_from_origin = np.array([1, tangent_parrallel_to_pt_tan])
        x_vector = np.array([1, 0]) # unit vector
        # ===============================================
        # Turn slope into an angle to calculate how much to modify the
        # original steering_cmd
        # ...............................................
        slope_sign = tangent_parrallel_to_pt_tan/(abs(tangent_parrallel_to_pt_tan) +self.AVOID_0_DIVISION)
        slope_angle = self.get_theta(tan_vector_from_origin, x_vector)

        # ===============================================
        # update the angle between <1,0> and <the vector_tangent> for the other three wheels
        # to know the offset needed for each wheel in four_wheel_steering
        # ...............................................
        offset_and_angle = lambda slope_theta : slope_theta +self.angle_pivot_wheel

        if( is_adjacent ):
            passed_inflection = True
        elif( is_sameside ):
            passed_inflection = False
        elif( is_across ):
            passed_inflection = passed_inflection

        if( not passed_inflection ):
            self.steer_offset_dict[wheel_name]= offset_and_angle( slope_sign*-1*slope_angle )
        elif( passed_inflection ):
            self.steer_offset_dict[wheel_name]= offset_and_angle(    THETA_SIGN*slope_angle )


    # check if the chassis is aligned with the pivot_wheel
    def chassis_aligned(self):
        return (round(self.angle_chassis, 1) == 0 and round(self.angle_pivot_wheel, 1) == 0)

    #######################################
    ##**********************************###
    ##       MATH HELPER FUNCTIONS      ###
    ##**********************************###
    #######################################

    def same_vect(self, prev_vect, curr_vect):
        if (round(prev_vect[0], 6) == round(curr_vect[0], 6)
                and round(prev_vect[1], 6) == round(curr_vect[1], 6)):
            return True
        return False

    # get tangent slope at x from pivot wheel circle equation
    def get_tangent_at(self, x, radius):
        tangent_slope = -1 * (x) / (math.sqrt(radius ** 2 - (x) ** 2))
        return tangent_slope

    def get_theta(self, a, b):
        return math.acos((a).dot(b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    # use rotation matrix to rotate coordinates by theta
    #
    # --- returns rotated vector
    def rotate( self , coord , theta ):
	    a , b = math.cos( theta ) , math.sin( theta )
	    rotation_matrix = np.array( [[ a , -b ],
								     [ b ,  a ]])
	    return rotation_matrix.dot(coord)

    # returns two missing sides, currently not in use
    def sine_law(self, angle_A, side_c, angle_C=math.pi / 2, sign=1):
        # ===============================================
        # want side a == x and side b == y
        # assuming side_c is the hypotenuse to a right triangle
        # returns [ side_a , side_b ]
        # ===============================================
        side_a = (side_c / math.sin(angle_C)) * math.sin(angle_A)
        side_b = (side_c / math.sin(angle_C)) * math.sin(math.pi / 2 - angle_A)
        return [side_a, side_b]
    # --------------------------------for testing ################################################
    # --------------------------------for testing ################################################
    # --------------------------------for testing ################################################
    def tester(self):
        pass


if __name__ == '__main__':
    try:
        RoverMotionController()
    except rospy.ROSInterruptException:
        pass
