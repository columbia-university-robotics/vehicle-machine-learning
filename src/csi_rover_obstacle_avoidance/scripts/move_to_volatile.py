#!/usr/bin/env python

import rospy
import tf
import math 
from collections import deque
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# from srcp2_msgs.msg import vol_sensor_msg
from csi_rover_localizer.srv import TargetCoordinate 
from csi_rover_obstacle_avoidance.srv import ObstacleAvoidance, ObstacleAvoidanceResponse

class VolatileFinder():
    def __init__(self):
        rospy.init_node('volatile_finder')

        rospy.on_shutdown(self.shutdown)
        self.rover_name = rospy.get_param('rover_name', "scout_1")
        rospy.loginfo("here-2")

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odometry = rospy.Subscriber('/debugging/gt_odom', Odometry, self.odometry_callback)
       # self.volatile = rospy.Subscriber('/scout_1/volatile_sensor', vol_sensor_msg, self.volatile_callback)
        self.srv = rospy.Service( "/" + self.rover_name + "/move_to_volatile" , ObstacleAvoidance , self.move_to_vol_callback )

        rospy.loginfo("here-1")
        self.current_position = Point(0,0,0)
        self.rotational_min_position = Point(0,0,0)
        self.potential_volatile_postion = Point(0,0,0)
        self.dist_volatile = 100000
        self.current_min_dist = 10000
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.target = (self.current_position.x,self.current_position.y)
        self.service_name = "/" + self.rover_name + "/target_coordinate"
        
        rospy.loginfo("here")
        self.coord_deque = deque([(self.current_position.x,self.current_position.y)])
        rospy.loginfo("here2")
        rospy.loginfo("move to volatile service is running")

    def move_to_vol_callback( self , request ):
		# main service logic

        rospy.loginfo("here3")
        try:
            self.turn_around()

            self.coord_deque.appendleft((self.rotational_min_position.x, self.rotational_min_position.y))
            self.coord_deque.appendleft((self.potential_volatile_postion.x, self.potential_volatile_postion.y))
            self.coord_deque.appendleft((self.current_position.x, self.current_position.y))

            return self.move_to_vol()
        except:
            return False


    def move_to_vol(self):
        
        self.target = self.coord_deque.popleft() 

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
                    if (self.coord_deque.count() == 0):
                        break
            except rospy.ServiceException, e:
                print( "Service call failed: %s"%e )
                rospy.wait_for_service( self.service_name )

            rate.sleep()

        return True



    def shutdown(self):
        rospy.loginfo('Stopping move_to_vol algorithm...')
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def odometry_callback(self, msg):
        # may want to check values < min and > max depending on performance
        self.current_position = msg.pose.pose.position
       # self.current_orientation = msg.pose.pose.orientation

        rotation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(rotation)
        # rospy.loginfo("ugh oh")
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

    def tight_right(self):
        rate = rospy.Rate(1) # 10hz
        forward = Twist()
        forward.linear.x = 20
        forward.linear.y = 0
        forward.linear.z = 0

        forward.angular.x = 0
        forward.angular.y = 0
        forward.angular.z = -15
        self.cmd_vel_pub.publish(forward)
        rospy.loginfo("we are hugging right")
        rate.sleep()

    def inch_forward(self):
        rate = rospy.Rate(1) # 10hz
        forward = Twist()
        forward.linear.x = 1
        forward.linear.y = 0
        forward.linear.z = 0

        forward.angular.x = 0
        forward.angular.y = 0
        forward.angular.z = 0
        self.cmd_vel_pub.publish(forward)
        rospy.loginfo("we are going straight")
        rate.sleep()

    def go_forward(self):
        rate = rospy.Rate(1) # 10hz
        forward = Twist()
        forward.linear.x = 80
        forward.linear.y = 0
        forward.linear.z = 0

        forward.angular.x = 0
        forward.angular.y = 0
        forward.angular.z = 0
        self.cmd_vel_pub.publish(forward)
        rospy.loginfo("we are going straight")
        rate.sleep()

    def turn_around(self):

        rospy.loginfo("now turning around")
        rate = rospy.Rate(1) 
        rate_for = rospy.Rate(100)

        min_dist = self.dist_volatile
        self.rotational_min_position = self.current_position

        home_base = self.yaw

        while True:
             # Keep turning right
            

            self.tight_right()
            if (self.dist_volatile < min_dist):
                min_dist = self.dist_volatile
                self.rotational_min_position = self.current_position

            rate.sleep()
            self.inch_forward()
            rate_for.sleep()

            if self.yaw - home_base < 0.01:
       
                rospy.loginfo("breaking turn")
                rate_for.sleep()
                break

        x = min_dist * math.cos(math.pi/4)
        y = min_dist * math.sin(math.pi/4)
        self.potential_volatile_postion.x = x + self.rotational_min_position.x
        self.potential_volatile_postion.y = y + self.rotational_min_position.y



    def detected(self):
        rospy.loginfo("WIP")


    def volatile_callback(self, msg):
               
        self.dist_volatile = msg.distance_to
        
    
    # deprecated 
    def volatile_finder(self):
        prev = self.dist_volatile
        while True:
            #rospy.loginfo('{}, {}, {}'.format(self.left_scan, self.center_scan, self.right_scan))
            current = self.dist_volatile
            # rospy.loginfo(current)
            self.go_forward()
            if(current - prev > 0.01):
                rospy.loginfo("Volatile Detected")
                self.turn_around()

            prev = current

def main():
    try:
        vf = VolatileFinder()
        rospy.spin()
    except:
        rospy.loginfo('volatileFinder node terminated')

if __name__ == "__main__":
    main()