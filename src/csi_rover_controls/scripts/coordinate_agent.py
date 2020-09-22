#!/usr/bin/env python
import sys          # for sys.float_info.min 
import time
import math         # for trig functions
import rospy
import numpy as np  # for dot products and normals
#import collections
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from csi_rover_controls.srv import AccelerationService 
from csi_rover_localizer import TargetCoordinateService # import service: target_coordinate/class to ease load on roscore



class CoordinateAgent():
  """
TO RUN FOR TESTING :

source devel/setup.bash
rosrun csi_rover_controls path_agent.py


  """
  AVOID_0_DIVISION = sys.float_info.min # AVOID_0_DIVISION = 2.2250738585072014e-308
  conversion_matrix = np.zeros([3,3])
  # define what theta to use explicit vs crab steering
  THETA_BOUND = math.pi/3
  def __init__(self):
    self.TCS = TargetCoordinateService()
    self.rover_name = rospy.get_param('rover_name')
    self.path_agent_started = False
    self.target = (0, 0)
    self.e_stop = False

    self.MAX_SPEED = 30

    #------------------------------
    #------- PUBLISHERS -----------
    #..............................
    #self.cmd_vel_pub = rospy.Publisher("/"+self.rover_name + "/cmd_vel", Twist, queue_size=1)
    self.cmd_vel_pub = rospy.Publisher("/"+self.rover_name + "/cmd_vel", Twist, queue_size=1)
    self.steer_state_pub = rospy.Publisher("/"+self.rover_name + "/steer_state", Float64,queue_size=1)
    self.path_msg = Twist()

    #------------------------------
    #------- SUBSCRIBERS ----------
    #..............................
    rospy.Subscriber("/debugging/gt_odom",Odometry , callback=self.odom_cb )
    #rospy.Subscriber("/" + self.rover_name + "/ekf_odom",Odometry , callback=self.odom_cb )
    #rospy.Subscriber("/" + self.rover_name + "/wheel_odom",Odometry , callback=self.odom_cb )
    rospy.Subscriber("/" + self.rover_name + "/imu",Imu , callback=self.imu_cb )

    rospy.init_node('path_agent')
    self.time_at_last_update = time.time()
    self.RATE = 30
    rate = rospy.Rate(self.RATE)

    #------------------------------
    #----- POSITION VARIABLES -----
    #..............................
    self.fws_on = True
    self.yaw_on = False   # determines whether to call algorithm
    self.global_direction = 0       # in radians
    self.relative_direction = 0     # in radians
    self.heading = 0    # in radians -- translation from quaternion
    self.curr_location = np.array([0,0])
    self.prev_location = np.array([0,0])
    self.relative_coord_of_target = np.array([0,0])
    #----- QUATERNION VARIABLES ---
    self.conversion_matrix = np.ones([3,3])
    self.angular_y = 0
    #------------------------------
    #----- VELOCITY VARIABLES -----
    #..............................

    self.curr_vel = 0 # updated in next_velocity(..)

    #------------------------------
    #-------START TESTING ----------
    #..............................
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
    rospy.Subscriber("/" + self.rover_name + "/joint_states", JointState, callback=self.joint_state_cb)
    self.joint_dict = {"fr_wheel_joint": {"lin_v": 0}, "fl_wheel_joint": {"lin_v": 0},
                       "br_wheel_joint": {"lin_v": 0}, "bl_wheel_joint": {"lin_v": 0}}
    # direction vector per wheel
    self.v_wheel_dict = {'bl': None, 'br': None, 'fl': None, 'fr': None}
    self.wheel_radius = rospy.get_param("wheel_radius", 0.275)
    #------- END TESTING ----------

    while not rospy.is_shutdown():

      if( self.TCS.off_requested or ( not self.TCS.target_location or not self.target )):#not self.path_agent_started  or 
        rospy.loginfo_throttle(2,  "Awaiting First Target..."+ " " + str(self.TCS.target_location))

      else:
        if( not self.TCS.off_requested and not self.TCS.current_is_target_location() ):
          self.target = self.TCS.target_location

        self.update_relative_direction( )
        self.determine_steering_type( self.relative_direction )
        #self.next_velocity()
        if( self.yaw_on and abs(self.relative_direction) < math.pi/12):
          self.yaw_on = False
          self.reset_path_msg_to(0)
          self.sleep_between_steering_change()
        elif( self.fws_on and self.THETA_BOUND < abs( self.relative_direction ) ):
          self.fws_on = False
          self.yaw_on = True

          self.reset_path_msg_to(0 , reset_angular = False  )
          self.reset_path_msg_to(1 , reset_linear  = False  )
          self.sleep_between_steering_change()
        else:#( not self.yaw_on ):
          self.cmd_vel_pub.publish(self.path_msg)
        #rospy.loginfo_throttle(2,  "current_target : "+ str(self.target) )


      rate.sleep()
  def reset_path_msg_to(self, val=0 , reset_linear = True , reset_angular = True  ):
    if( reset_linear ): 
      self.path_msg.linear.x = self.path_msg.linear.y = self.path_msg.linear.z = val
    if( reset_angular ):
      self.path_msg.angular.x= self.path_msg.angular.y= self.path_msg.angular.z= val
    
  def sleep_between_steering_change(self, val=0): 
    self.cmd_vel_pub.publish(self.path_msg)
    rospy.sleep(3.)

  def update_relative_direction( self ):
    # heading is updated in imu_cb
    self.relative_coord_of_target = self.rotate( self.target-self.curr_location , -self.heading )   

    if( round(self.curr_location[0],0)==round(self.target[0],0)
    and round(self.curr_location[1],0)==round(self.target[1],0)):

      if( not self.TCS.off_requested ):
        #service version
        self.target = self.TCS.target_location
      
    self.relative_direction = math.atan2( self.relative_coord_of_target[1] ,
                        self.relative_coord_of_target[0] )



  # determine steering type and update self.path_msg parameters accordingly
  # THETA_BOUND == radian number
  #
  #   THETA_BOUND <  abs( relative_theta )   ---------> set ang.z and lin.x
  # ---- EXPLICIT
  #
  #   THETA_BOUND >= abs( relative_theta )   ---------> set lin.x and lin.y
  # ---- CRAB 
  def determine_steering_type(self , relative_theta ):

    # reset cmd_vel message
    self.reset_path_msg_to(0)
    rospy.loginfo_throttle(2,"relative coord : "+str( self.relative_coord_of_target)+" crab : "+str( round(relative_theta ,3)))

    # check how at what angle the target location is at
    if( self.yaw_on ):#or self.THETA_BOUND < abs( relative_theta ) ): 
      # EXPLICIT
      self.path_msg.angular.x = 1
      self.path_msg.angular.z = relative_theta
      self.path_msg.linear.x  = 30

      if( int(rospy.get_time()) % 5 == 0 ):
        self.steer_state_pub.publish(0)
      #rospy.loginfo_throttle(2,"explicit : "+str(abs( round(relative_theta,3)) ) + str(self.path_msg.linear.x))
    else:
      #rospy.loginfo_throttle(2,"crab : "+str(abs( round(relative_theta ,3))))
      # CRAB
      self.fws_on = True
      self.path_msg.linear.x  =  20#self.curr_vel 
      self.path_msg.angular.x = 0
      self.path_msg.angular.z = relative_theta
      STRAIGHT , SOFT_LEFT , HARD_LEFT , SOFT_RIGHT , HARD_RIGHT = 1 , 2 , 3 , 4 , 5
      if( round(relative_theta, 1) == 0 ):
        self.steer_state_pub.publish( STRAIGHT )

      elif( round(relative_theta, 1) < math.pi/6 and 0 < relative_theta ):
        self.steer_state_pub.publish( SOFT_LEFT ) 

      elif( round(relative_theta, 1) > math.pi/6 and 0 < relative_theta ):
        self.steer_state_pub.publish( HARD_LEFT ) 

      elif( round(relative_theta, 1) > -math.pi/6 and relative_theta < 0 ):
        self.steer_state_pub.publish( SOFT_RIGHT ) 

      elif( round(relative_theta, 1) < -math.pi/6 and relative_theta < 0 ):
        self.steer_state_pub.publish( HARD_RIGHT ) 



  #######################################
  ##**********************************###
  ##       MATH HELPER FUNCTIONS      ###
  ##**********************************###
  #######################################

  # --- returns theta between a and b
  def get_theta(self , a , b ):
    return math.acos( (a).dot(b)/(np.linalg.norm(a)*np.linalg.norm(b)) )

  # use rotation matrix to rotate coordinates by theta
  #
  # --- returns rotated vector
  def rotate( self , coord , theta ):
    a , b = math.cos( theta ) , math.sin( theta )
    rotation_matrix = np.array( [[ a , -b ],
                   [ b ,  a ]])
    return rotation_matrix.dot(coord)


  #######################################
  ##**********************************###
  ##       MAIN heading UPDATER       ###
  ##**********************************###
  #######################################
  
  #======================================
  #========== MAIN CALL BACK ============
  #======================================
  # Get the imu sensor data and convert
  # the quaternion to heading information
  # 
  # Updates heading angle 
  def imu_cb( self , data ):
    _w , _x , _y , _z = data.orientation.w ,data.orientation.x , data.orientation.y , data.orientation.z
    #a_y = data.angular_velocity.y
    #print( "imu_cb Orientation.z == ", _z )
    """
    # see bottom in case not given the orientation quaternion
    """

    self.conversion_matrix[0] = [1 - 2*( _y**2 + _z**2) ,2*(_x*_y + _w*_z),  2*(_x*_z -_w*_y)]
    self.conversion_matrix[1] = [2*(_x*_y - _w*_z) , 1 - 2*(_x**2 + _z**2), 2*(_y*_z + _w*_x)]
    self.conversion_matrix[2] = [2*(_x*_z + _w*_y) , 2*(_y*_z - _w*_x), 1 - 2*(_x**2 + _y**2)]
    
    # FLIPPED INDEXES TO MATCH THE GAMA MATRIX BASED ON THE CHROBOTICS MATRIX

    #----------- YAW --------------
    #print( math.atan2( self.conversion_matrix[0,1],self.conversion_matrix[0,0]))
    #self.heading = math.atan2( self.conversion_matrix[0,1],self.conversion_matrix[0,0])

    # should be roll based chrobotics algo but I'm using gama matrix
    #print( math.atan2( self.conversion_matrix[1,2],self.conversion_matrix[2,2]))

    # should be pitch based chrobotics algo but I'm using gama matrix
    # print( -math.asin( self.conversion_matrix[0,2]))
    #self.angular_y = a_y

    # ======================================
    # ========== JOINTCALL BACK ============
    # ======================================
    # Get the wheel joint states
    # Updates direction vectors for each wheel
    #       ( 1 , 0 , 0 ) is forward
  def joint_state_cb(self, data):
    """
    data.name || data.position ||  data.velocity
    """
      
    for i in range(len(data.name)):
      self.joint_dict[data.name[i]] = {"pos": data.position[i],
                                     "vel": data.velocity[i],
                                     "lin_v": data.velocity[i] * self.wheel_radius}



    # ======================================
    # ========== NEXT VEL       ============
    # ======================================
    #  
  def next_velocity( self ):

    if( self.curr_vel == 0 or not  self.joint_dict["fr_wheel_joint"]["lin_v"] ): #and not self.e_stop ):
      magnitude = math.sqrt( self.target[0]**2 + self.target[1]**2 )
      if( 1 < magnitude ):
        self.curr_vel = 1
      self.curr_vel = magnitude
    else:
      self.curr_vel = self.curr_vel # TODO class
    service_name = "/acceleration_service"
    try:
      #target_service = rospy.ServiceProxy( service_name , AccelerationService)
      arm_angle = min(self.joint_dict["fr_arm_joint"]["pos"],self.joint_dict["fl_arm_joint"]["pos"] )
      if( .05 < arm_angle ):
        arm_angle = .05 

      self.curr_vel =  self.new_vel( arm_angle ) 

    except rospy.ServiceException, e:
      print( "Service call failed: %s"%e )

    return self.curr_vel

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

#--------------------------------------------------------------------------------------------------
# TESTING
  def new_vel( self , front_arm_angle):
    pi_const = math.pi

    e_to_x_velocity = 1/(1+math.exp( self.curr_vel / 7*math.exp(1.0) ))
    velocity_result = 100 - 200*( e_to_x_velocity )
    base_slow_func  = -velocity_result*( 2/pi_const )
    turning_result  = base_slow_func*( abs(self.relative_direction)*( 1/(pow(pi_const,4)))) 
    if( front_arm_angle > 0.013  ):
      front_arm_angle = 0.013
    if( front_arm_angle > -0.037  ):
      f_arm_result = base_slow_func*( abs(front_arm_angle+0.037)*( 135/pi_const)) 
    else :
      f_arm_result = 0 
    return abs(velocity_result + turning_result + f_arm_result)









if __name__ == '__main__':
  try:
    CoordinateAgent()
  except rospy.ROSInterruptException: 
    pass

