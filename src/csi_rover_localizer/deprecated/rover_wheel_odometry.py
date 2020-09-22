#!/usr/bin/env python
import sys  # for sys.float_info.min
import math  # for trig functions
import rospy
import numpy as np  # for dot products and normals
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry  # for tests

"""
crater is a great testing location
-40.000000, -30

...........
"""


class WheelOdom():
    """
TO RUN FOR TESTING :

source devel/setup.bash
rosrun csi_rover_controls rover_wheel_odometry.py
    """

    AVOID_0_DIVISION = sys.float_info.min  # AVOID_0_DIVISION = 2.2250738585072014e-308
    conversion_matrix = np.zeros([3, 3])

    def __init__(self):
		self.r_name = "/"+rospy.get_param('rover_name', "scout_1")
		self.wheel_radius = rospy.get_param( self.r_name+"/wheel_radius")
		self.wheel_separation_length = rospy.get_param( self.r_name+"/wheel_separation_length")
		self.wheel_separation_width= rospy.get_param( self.r_name+"/wheel_separation_width")
		self.icr_radius = rospy.get_param( self.r_name+"/icr_radius")  

		# ------------------------------
		# -----   ODOM VARIABLES   -----
		# ..............................
		self.wheel_odom = None
		self.bearing = 0  # in radians -- translation from quaternion
		self.pitch = 0
		"""
					STRUCTURE
		self.joint_dict[key] == 
			{ "pos": data.position[i] ,
			  "vel": data.velocity[i] ,
			"lin_v": data.velocity[i]*self.wheel_radius }

						joint_dict keys
		bl_arm_joint , bl_steering_arm_joint , bl_wheel_joint ,
		br_arm_joint , br_steering_arm_joint , br_wheel_joint ,
		fl_arm_joint , fl_steering_arm_joint , fl_wheel_joint ,
		fr_arm_joint , fr_steering_arm_joint , fr_wheel_joint ,
		sensor_joint
		"""

		self.joint_dict = {"fr_wheel_joint": {"lin_v": 0}, "fl_wheel_joint": {"lin_v": 0},
							"br_wheel_joint": {"lin_v": 0}, "bl_wheel_joint": {"lin_v": 0}}
		# direction vector per wheel
		self.v_wheel_dict = {'bl': None, 'br': None, 'fl': None, 'fr': None}

		# ----- ICR VARIABLES ----------
		self.pivot_wheel_linear_vel = 0  # # updated in cmd_vel_cb to get steering_cmd
		self.pos_delta = None
		self.pos_delta_with_z = None
		self.time_delta = 0
		self.theta_delta = 0  # around ICR
		self.steering_cmd = 0
		self.using_explicit_steer = False

		# v_* vectors for use in *_rotation methods
		v_front_neg = np.array([0, self.wheel_separation_width])  # steering_cmd is neg
		v_front_pos = -1 * v_front_neg  # steering_cmd is pos
		v_adj = -1 * np.array([self.wheel_separation_length, 0])
		self.v_center_neg = 0.5 * (v_front_neg + v_adj)  # steering_cmd is neg
		self.v_center_pos = 0.5 * (v_front_pos + v_adj)  # steering_cmd is pos

		self.v_rover = 0
		self.v_icr = np.array([self.icr_radius, 0])

		# ----- QUATERNION VARIABLES ---
		self.conversion_matrix = np.ones([3, 3])





		# ------------------------------
		# ------- PUBLISHERS -----------
		# ..............................

		self.twist_msg = Twist()
		self.wheel_odom_pub = rospy.Publisher(self.r_name + "/wheel_odom", Odometry, queue_size=10)

		# ------------------------------
		# ------- SUBSCRIBERS ----------
		# ..............................
		#rospy.Subscriber("/debugging/gt_odom", Odometry, callback=self.ground_truth_cb)
		#rospy.Subscriber("/debugging/init_pose", Pose, callback=self.init_pose_cb)

		rospy.Subscriber(self.r_name + "/cmd_vel", Twist, callback=self.cmd_vel_cb)
		rospy.Subscriber(self.r_name + "/imu", Imu, callback=self.imu_cb)
		rospy.Subscriber(self.r_name + "/joint_states", JointState, callback=self.joint_state_cb)


		# ------------------------------
		# -------START TESTING ----------
		# ..............................
		#self.init_pose = rospy.get_param("initial_pose", [0, 0])
		#self.init_pose = None
		#if rospy.has_param(self.r_name + '/target'):
		#    pass  # rospy.set_param(self.r_name + '/target',{"x":1,"y":1})

		# ------- END TESTING ----------

		rospy.init_node('rover_wheel_odometry')

		self.RATE = 30
		rate = rospy.Rate(self.RATE)
		self.prev_time = rospy.get_time()

		initial_position_param = "/localization/rover_wheel_odometry/initial_pose"
		while( self.wheel_odom is None ): 
			rospy.loginfo_throttle(2, "Waiting for Initial Position parameter.")
			if rospy.has_param( initial_position_param ):
				x,y = rospy.get_param(initial_position_param)
				self.wheel_odom = np.array([x,y])
				#self.wheel_odom_z = z

		while not rospy.is_shutdown():

			self.radian_calculations('start')
			self.v_rover = self.pre_rotation()

			self.time_delta = rospy.get_time() - self.prev_time

			self.radian_calculations('end')
			self.pos_delta = self.post_rotation(self.v_rover)
			self.check_explicit_steer()
			if( not self.using_explicit_steer ):
				self.update_wheel_odom()
			self.publish_wheel_odom()
			print_msg = "\n\nwheel odom : " + str(np.around(self.wheel_odom, 4))+str(self.using_explicit_steer)+"\n\n"
			"""
			print_msg +=str( np.around(self.pos_delta ,4))
			print_msg += "bearing : "+str(round(self.bearing,4))
			print_msg += "steering : "+ str(round(self.steering_cmd,4))
			"""
			rospy.loginfo_throttle(1, print_msg)
			#print(str(self.joint_dict)+"\n")
			self.prev_time = rospy.get_time()
			rate.sleep()






    # ======================================
    # 			RADIAN CALCULATIONS
    # ======================================
    def radian_calculations(self, mode):
        if (mode == "start"):
            self.theta_2pi = ((self.icr_radius) * 2 * math.pi) / (
                    abs(self.pivot_wheel_linear_vel) + self.AVOID_0_DIVISION)
        elif (mode == "end"):
            self.theta_delta = 2 * math.pi * (self.time_delta / self.theta_2pi)

    # ======================================
    # 			PRE ROTATION
    # ======================================
    #	1 :
    # self.v_rover --- defines rover center
    #
    def pre_rotation(self):
        """
        self.v_center_neg # steering_cmd is neg
        self.v_center_pos # steering_cmd is pos
		"""
        if (0 < self.steering_cmd):
            # steering_cmd is pos
            v_center_rotated = self.rotate2(self.v_center_pos, -self.steering_cmd)
        else:
            # steering_cmd is neg
            # or driving straight
            v_center_rotated = self.rotate2(self.v_center_neg, self.steering_cmd + math.pi)
        return v_center_rotated + self.v_icr

    # ======================================
    # 			POST ROTATION
    # ======================================
    #	1 :
    # rotates v_rover_start by the
    # traveled radians around ICR
    # to equal v_rover_end
    #	2 :
    # make v_rover_delta
    #	3 :
    # make pos_delta with ( 1 , 0 )
    # defining forward direction
    #
    def post_rotation(self, v_rover_start):
        v_rover_end = self.rotate2(v_rover_start, self.theta_delta)
        v_rover_delta = v_rover_end - v_rover_start
        if (0 < round(self.steering_cmd+.2, 0)):
            # steering_cmd is pos
            pos_delta_y_forward = self.rotate2(v_rover_delta, self.steering_cmd)
        elif (round(self.steering_cmd-.2, 0) < 0):
            # steering_cmd is neg
            pos_delta_y_forward = self.rotate2(v_rover_delta, self.steering_cmd - math.pi)
        else:
            # is driving straight
            pos_delta_y_forward = np.array([0, self.pivot_wheel_linear_vel]) * self.time_delta
        # normalize to ( 1, 0 ) as forward
        return self.rotate2(pos_delta_y_forward, -math.pi / 2)

    # ======================================
    # 			WHEEL ODOM
    # ======================================
    #	1 :
    # rotates pos_delta by the rover bearing
    # to get true new location
    #	2 :
    # adds pos_delta to global coordinates
    def update_wheel_odom(self):
        # pitch with y being forward
        #self.pos_delta_with_z = np.array( [self.pos_delta[0],self.pos_delta[1],0] )		
        #self.pos_delta_with_z = self.rotate3( self.pos_delta_with_z, self.pitch, axis='x')
        #self.pos_delta = self.pos_delta_with_z[:2]

        pos_delta_rotated = self.rotate2(self.pos_delta, self.bearing)
        self.wheel_odom = self.wheel_odom + pos_delta_rotated
        #self.wheel_odom_z = self.wheel_odom_z + self.pos_delta_with_z[2]

    #######################################
    ##**********************************###
    ##       CALL BACK FUNCTIONS        ###
    ##**********************************###
    #######################################

    # ======================================
    # ========== MAIN CALL BACK ============
    # ======================================
    # Get the wheel joint states
    # Updates direction vectors for each wheel
    # 			( 1 , 0 , 0 ) is forward
    def joint_state_cb(self, data):
        """
		data.name || data.position ||  data.velocity
		"""

        def update_wheel_dir_v(wheel):
            # TODO figure out :
            # if *_arm_joint position value is affected by back vs front and left vs right
            # confirm *_steering_arm_joint position gives expected rotation
            direction_vector = np.array([0, 0, 0])
            direction_vector[0] = self.joint_dict[wheel + "_wheel_joint"]["lin_v"]
            self.v_wheel_dict[wheel] = self.rotate3(direction_vector,
                                                    self.joint_dict[wheel + "_steering_arm_joint"]["pos"], 'z')
            self.v_wheel_dict[wheel] = self.rotate3(direction_vector, self.joint_dict[wheel + "_arm_joint"]["pos"], 'x')
			
        for i in range(len(data.name)):
            self.joint_dict[data.name[i]] = {"pos": data.position[i],
                                             "vel": data.velocity[i],
                                             "lin_v": data.velocity[i] * self.wheel_radius}
        for k, v in self.v_wheel_dict.items():
            # update self.v_wheel_dict with the wheel vectors
            prev_v = self.v_wheel_dict[k]
            update_wheel_dir_v(k)
            if (not (prev_v == self.v_wheel_dict[k]).all):
                self.prev_time = rospy.get_time()


    # ======================================
    # ========== IMU  CALL BACK ============
    # ======================================
    # Get the imu sensor data and convert
    # the quaternion to bearing information
    #
    # Updates bearing angle
    def imu_cb(self, data):
        _w, _x, _y, _z = data.orientation.w, data.orientation.x, data.orientation.y, round(data.orientation.z, 4)

        self.conversion_matrix[0] = [1 - 2 * (_y ** 2 + _z ** 2), 2 * (_x * _y + _w * _z), 2 * (_x * _z - _w * _y)]
        self.conversion_matrix[1] = [2 * (_x * _y - _w * _z), 1 - 2 * (_x ** 2 + _z ** 2), 2 * (_y * _z + _w * _x)]
        self.conversion_matrix[2] = [2 * (_x * _z + _w * _y), 2 * (_y * _z - _w * _x), 1 - 2 * (_x ** 2 + _y ** 2)]
        # ----------- YAW --------------
        # print( math.atan2( self.conversion_matrix[0,1],self.conversion_matrix[0,0]))
        self.bearing = round(math.atan2(self.conversion_matrix[0, 1], self.conversion_matrix[0, 0]), 2)

        # should be roll based chrobotics algo but I'm using gama matrix
        #print( math.atan2( self.conversion_matrix[1,2],self.conversion_matrix[2,2]))

        # should be pitch based chrobotics algo but I'm using gama matrix
        # print( -math.asin( self.conversion_matrix[0,2]))
        self.pitch = round(-math.asin( self.conversion_matrix[0,2]),1)

    # ======================================
    # ======== CMD_VEL CALL BACK ===========
    # ======================================
    # Determine steering angle
    def cmd_vel_cb(self, data):
        print_msg = ""
        self.steering_cmd = math.atan(data.linear.y / (data.linear.x + self.AVOID_0_DIVISION))
        if (0.0 == round(self.steering_cmd, 6)):
            # driving straight ( forward ), use rear wheel w. minVel b.c. it has more traction
            self.pivot_wheel_linear_vel = max([self.joint_dict["bl_wheel_joint"]["lin_v"],
                                               self.joint_dict["br_wheel_joint"]["lin_v"]])
        elif (0 < self.steering_cmd):
            # pivot wheel is right front
            self.pivot_wheel_linear_vel = self.joint_dict["fr_wheel_joint"]["lin_v"]
            #print_msg += "fr_wheel_joint "
        else:
            # pivot wheel is left front
            self.pivot_wheel_linear_vel = self.joint_dict["fl_wheel_joint"]["lin_v"]
            #print_msg += "fl_wheel_joint "


    #######################################
    ##**********************************###
    ##       MATH HELPER FUNCTIONS      ###
    ##**********************************###
    #######################################

    # --- returns theta between a and b
    def get_theta(self, a, b):
        return math.acos((a).dot(b) / (np.linalg.norm(a) * np.linalg.norm(b)))

    # use 2x2 rotation matrix to rotate coordinates by theta
    #
    # --- returns rotated vector
    def rotate2(self, coord, theta, state="position"):
        a, b = math.cos(theta), math.sin(theta)
        if (state == "derivative_theta"):  # formula check through VVV
            # http://www8.tfe.umu.se/courses/elektro/RobotControl/Lecture06_5EL158.pdf
            rotation_matrix = np.array([[-b, -a],
                                        [a, -b]])
            return rotation_matrix.dot(coord)
        rotation_matrix = np.array([[a, -b],
                                    [b, a]])
        return rotation_matrix.dot(coord)

    # use 3x3 rotation matrix to rotate coordinates by theta
    #
    # --- returns rotated vector
    def rotate3(self, coord, theta, axis='z'):
        a, b = math.cos(theta), math.sin(theta)
        rotation_matrix = None
        if axis == 'z':
            rotation_matrix = np.array([[a, -b, 0],
                                        [b, a, 0],
                                        [0, 0, 1]])
        elif axis == 'y':
            rotation_matrix = np.array([[a, 0, -b],
                                        [0, 1, 0],
                                        [b, 0, a]])
        elif axis == 'x':
            rotation_matrix = np.array([[1, 0, 0],
                                        [0, a, -b],
                                        [0, b, a]])
        return rotation_matrix.dot(coord)

    #######################################
    ##**********************************###
    ##       PUBLISHER FUNCTIONS        ###
    ##**********************************###
    #######################################



    def publish_wheel_odom(self):
	    odom_msg = Odometry()
	    odom_msg.header = Header()
	    odom_msg.header.stamp = rospy.Time.now()
	    odom_msg.header.frame_id = "odom"
	    odom_msg.child_frame_id = "scout_1_tf/base_footprint"
	    odom_msg.pose = PoseWithCovariance()
	    odom_msg.pose.pose = Pose()

	    odom_msg.pose.pose.position.x = self.wheel_odom[0] 
	    odom_msg.pose.pose.position.y = self.wheel_odom[1] 
	    #odom_msg.pose.pose.position.z = self.wheel_odom_z

	    odom_msg.twist = TwistWithCovariance()
	    odom_msg.twist.twist = Twist()

	    self.wheel_odom_pub.publish(odom_msg)


    def check_explicit_steer(self):
		if( (self.joint_dict["fr_wheel_joint"]["lin_v"] < 0
		and self.joint_dict["br_wheel_joint"]["lin_v"] < 0
		and self.joint_dict["fl_wheel_joint"]["lin_v"] > 0
		and self.joint_dict["bl_wheel_joint"]["lin_v"] > 0 ) 
		or\
		(self.joint_dict["fr_wheel_joint"]["lin_v"] > 0
		and self.joint_dict["br_wheel_joint"]["lin_v"] > 0
		and self.joint_dict["fl_wheel_joint"]["lin_v"] < 0
		and self.joint_dict["bl_wheel_joint"]["lin_v"] < 0 ) 
		or\
		(round(self.joint_dict["fr_wheel_joint"]["lin_v"],6) == 0
		and round(self.joint_dict["br_wheel_joint"]["lin_v"],6) == 0
		and round(self.joint_dict["fl_wheel_joint"]["lin_v"],6) == 0
		and round(self.joint_dict["bl_wheel_joint"]["lin_v"],6) == 0 ) ):
			self.using_explicit_steer = True
		else:
			self.using_explicit_steer = False

    # --------------------------------for testing ################################################
    # --------------------------replace with odom later ##########################################
    def ground_truth_cb(self, data):
        # self.x_rover = data.pose.pose.position.x
        # self.y_rover = data.pose.pose.position.y
        self.curr_location = np.array([data.pose.pose.position.x, data.pose.pose.position.y])


if __name__ == '__main__':
    try:
        WheelOdom()
    except rospy.ROSInterruptException:
        pass


"""
    def publish_wheel_vectors(self):
        # TODO
        # if want to use ROS EKF
        # ---- ready -- imu
        # ---- ready -- odom estimation via ICR  ( no Z component )
        # ----?ready?-- vo
        " ""
 <launch>
  <node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf">
    <param name="output_frame" value="ekf_odom"/>
    <param name="freq" value="30.0"/>
    <param name="sensor_timeout" value="1.0"/>
    <param name="odom_used" value="true"/>
    <param name="imu_used" value="true"/>
    <param name="vo_used" value="true"/>
    <param name="debug" value="false"/>
    <param name="self_diagnose" value="false"/>

 	<remap from="odom" to="/wheel_odom" />    
 	<remap from="imu_data" to="/imu" />    
  </node>
 </launch>
		" ""
        #
        #
        #
        # if want to make own ekf --- https://www.researchgate.net/post/How_do_we_determine_noise_covariance_matrices_Q_R
        # REALLY GOOD ---- pg 41 https://www.researchgate.net/profile/Sami_Aldalahmeh/post/How_do_we_determine_noise_covariance_matrices_Q_R/attachment/5e369ae73843b06506d6858f/AS%3A854067228184576%401580636903183/download/Discrete-time+Kalman+filter.pdf
        # alright --------http://www.cs.cmu.edu/~16831-f14/notes/F10/16831_lecture20_21_zlamb_jlibby/16831_lecture20_21_zlamb_jlibby.pdf
        # for covariance
        # Q == number of states
        #	----high acceleration thus --> _arm_joint moves and some wheels are in the air
        #	----terrain change    thus --> _arm_joint moves and all wheels are grounded
        # R == number of inputs
        #	----estimated x
        #	----estimated y
        #	----TODO estimated z
        #	----individual wheel linear velocities
        #	----arm joint positions
        #	----steering positions???
        #
        # if want to reuse premade and cite https://www.researchgate.net/publication/323998110_Extended_Kalman_Filter_implementation_in_ROS_using_Python
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = self.r_name
        odom_msg.pose = PoseWithCovariance()
        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position.x = self.wheel_odom[0]
        odom_msg.pose.pose.position.y = self.wheel_odom[1]
        odom_msg.twist = TwistWithCovariance()
        odom_msg.twist.twist = Twist()

    # self.wheel_odom_pub.publish(odom_msg)


"""
