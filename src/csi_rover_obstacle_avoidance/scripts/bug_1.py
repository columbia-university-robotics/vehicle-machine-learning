#!/usr/bin/env python
# Bug 1 Algo
# This is now just a service
# When called it will do a full loop around an obstacle
# We do NOT do obstacle detection
# See CMU paper: 
# 
# Maintainer: Mikey (Chris) Calloway

import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from csi_rover_obstacle_avoidance.srv import ObstacleAvoidance, ObstacleAvoidanceResponse

class Bug1():
    def __init__(self):

        rospy.init_node( "bug1" )

        # Get rover name
        self.rover_name = rospy.get_param('rover_name', "scout_1")

        self.cmd_vel_pub = rospy.Publisher('/scout_1/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scout_1/laser/filtered', LaserScan, self.scan_callback)
        self.odometry = rospy.Subscriber('/debugging/gt_odom', Odometry, self.odometry_callback)
        self.srv = rospy.Service( "/" + self.rover_name + "/Bug_1" , ObstacleAvoidance , self.bug1_srv_callback )
        self.rover_name = rospy.get_param('rover_name')

        self.left_scan = float('inf')
        self.right_scan = float('inf')
        self.center_scan = float('inf')


        self.current_position = Point(0,0,0)
        self.position_memory = Point(0,0,0)
        self.obstacle_memory = Point(0,0,0)
        self.min_distance = 1000 #larger than any distance on this map
        self.goal = Point(0,0,0) # set by service call

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("bug 1 service is running")

    def bug1_srv_callback( self , request ):
		# main service logic

        # set the goal for bug
        self.goal.x = request.goal_x
        self.goal.y = request.goal_y
        try:
           
            bug_1_complete = self.circle_obstacle()
            success = ObstacleAvoidanceResponse(bug_1_complete, self.current_position.x, self.current_position.y)
            return success
        except:
            failure = ObstacleAvoidanceResponse(False, self.current_position.x, self.current_position.y)
            return failure


    def shutdown(self):
        rospy.loginfo('Stopping bug1 algorithm...')
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    # Calc Distance (Euclidean, 2 dimensional)
    def calc_distance(self, x_pos, x_goal, y_pos, y_goal):
        rospy.loginfo("Calcing distance")
        delta_goal_x = x_pos - x_goal
        delta_goal_y = y_pos - y_goal
        delta_x_square = delta_goal_x ** 2
        delta_y_square = delta_goal_y ** 2

        dist_square = delta_x_square + delta_y_square

        dist = math.sqrt(dist_square)
        rospy.loginfo(dist)
        return dist


    # turn rover to align with the goal 
    # after algo has finished
    def turn_theta_goal(self, current_position, goal):   
        delta_x = current_position.x - goal.x
        np.absolute(delta_x)
        delta_y = current_position.y - goal.y
        np.absolute(delta_y)

        theta = np.arctan(delta_y/delta_x)

        rate = rospy.Rate(1) # 10hz
        forward = Twist()
        forward.linear.x = 40
        forward.linear.y = 0
        forward.linear.z = 0

        forward.angular.x = 0
        forward.angular.y = 0
        forward.angular.z = theta
        self.cmd_vel_pub.publish(forward)
        rospy.loginfo("we are turning by theta")
        rate.sleep()


    
    def scan_callback(self, msg):
        # may want to check values < min and > max depending on performance
        # num_scans = len(msg.ranges)
        # self.left_scan = min(msg.ranges[2*num_scans//3:num_scans])
        # self.right_scan = min(msg.ranges[0:num_scans//3])
        # self.center_scan = min(msg.ranges[num_scans//3:2*num_scans//3])


        # Old callback
        self.left_scan = msg.ranges[-1]
        self.right_scan = msg.ranges[0]
        self.center_scan = msg.ranges[len(msg.ranges) / 2]

    def odometry_callback(self, msg):
        # may want to check values < min and > max depending on performance
        self.current_position = msg.pose.pose.position
        # rospy.loginfo("ugh oh")

       # move forward at a medium speed
    def stop(self):
        rate = rospy.Rate(1) # 10hz
        forward = Twist()
        forward.linear.x = 0
        forward.linear.y = 0
        forward.linear.z = 0

        forward.angular.x = 0
        forward.angular.y = 0
        forward.angular.z = 0
        self.cmd_vel_pub.publish(forward)
        rospy.loginfo("we are stoppping")
        rate.sleep()

    # move forward at a medium speed
    def go_forward(self):
        rate = rospy.Rate(1) # 10hz
        forward = Twist()
        forward.linear.x = 40
        forward.linear.y = 0
        forward.linear.z = 0

        forward.angular.x = 0
        forward.angular.y = 0
        forward.angular.z = 0
        self.cmd_vel_pub.publish(forward)
        rospy.loginfo("we are going straight")
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


    def hug_right(self):
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


    def tight_right(self):
        rate = rospy.Rate(1) # 10hz
        forward = Twist()
        forward.linear.x = 10
        forward.linear.y = 0
        forward.linear.z = 0

        forward.angular.x = 0
        forward.angular.y = 0
        forward.angular.z = -10
        self.cmd_vel_pub.publish(forward)
        rospy.loginfo("we are hugging right")
        rate.sleep()

    def tight_left(self):
        rate = rospy.Rate(1) # 10hz
        forward = Twist()
        forward.linear.x = 10
        forward.linear.y = 0
        forward.linear.z = 0

        forward.angular.x = 0
        forward.angular.y = 0
        forward.angular.z = 10
        self.cmd_vel_pub.publish(forward)
        rospy.loginfo("we are hugging right")
        rate.sleep()

    # hug is a very small turn
    def hug_left(self):
        rate = rospy.Rate(1) # 10hz
        forward = Twist()
        forward.linear.x = 20
        forward.linear.y = 0
        forward.linear.z = 0

        forward.angular.x = 0
        forward.angular.y = 0
        forward.angular.z = 15
        self.cmd_vel_pub.publish(forward)
        rospy.loginfo("we are hugging left")
        rate.sleep()


    # Veer is a very wide turn
    def veer_left(self):
        rospy.loginfo("we are veering left")
        rate = rospy.Rate(0.3) 
        rate_for = rospy.Rate(100)

        while True:
             # Keep turning right
            while self.right_scan > 5:

                self.tight_left()
                rate.sleep()
                self.inch_forward()
                rate.sleep()

            rospy.loginfo("breaking turn")
            rate.sleep()
            break
       
    
        rate.sleep()



    def turn_around(self):

        rospy.loginfo("now turning around")
        rate = rospy.Rate(0.3) 
        rate_for = rospy.Rate(100)

        while True:
             # Keep turning right
            while self.left_scan > 5:

                self.hug_right()
                rate.sleep()
                self.inch_forward()
                rate.sleep()
       
            rospy.loginfo("breaking turn")
            rate.sleep()
            break


        

    # Circles an obstacle
    def circle_obstacle(self):

        # rates are used for different
        # sleep rates
        rate = rospy.Rate(0.3) 
        shift_rate = rospy.Rate(0.1) 

        # First veer left so we are forced
        # to go clockwise around the obstacle
        self.veer_left()
        rate.sleep()
        rospy.loginfo("leaving veer left")
        self.set_memory()
        self.go_forward()
        rate.sleep()
       
     
        # Make a full loop around the rock
        cont = True


        scan_threshold = 5
        delta_threshold = 2.2
        scan_min_threshold = 2
        while cont:

            # General idea:
            # 'Fall through' these
            # while statements
            # which will make rover move
            # in a circular path around obstacle

            # If we turn too far way, hug to the
            # right again
            if self.right_scan > scan_threshold:

                # rospy.loginfo(self.current_position)
                rospy.loginfo("Do we break here1????")

                self.tight_right()
                rate.sleep()
                self.go_forward()
                rate.sleep()


                dist = self.calc_distance(self.current_position.x, self.goal.x, self.current_position.y, self.goal.y)
                if (dist < self.min_distance):
                    self.obstacle_memory = self.current_position
                    self.min_distance = dist

                delta_x = self.current_position.x - self.position_memory.x
                delta_y = self.current_position.y - self.position_memory.y
                delta_x = np.absolute(delta_x)
                delta_y = np.absolute(delta_y)

                rospy.loginfo(delta_x)
                rospy.loginfo(delta_y)

                if (delta_x < delta_threshold and delta_y < delta_threshold ):
                    rospy.loginfo("Breaking")
                    cont = False
                    break
                
                continue


            # other wise just keep going forward
            if self.right_scan <= scan_threshold and self.right_scan > scan_min_threshold:

                # rospy.loginfo(self.current_position)
                rospy.loginfo("Do we break here2????")

                
                self.go_forward()
                rate.sleep()

                dist = self.calc_distance(self.current_position.x, self.goal.x, self.current_position.y, self.goal.y)
             
                if (dist < self.min_distance):
                    self.obstacle_memory = self.current_position
                    self.min_distance = dist

                delta_x = self.current_position.x - self.position_memory.x
                delta_y = self.current_position.y - self.position_memory.y
                delta_x = np.absolute(delta_x)
                delta_y = np.absolute(delta_y)

                rospy.loginfo(delta_x)
                rospy.loginfo(delta_y)

                if (delta_x < delta_threshold and delta_y < delta_threshold ):
                    rospy.loginfo("Breaking")
                    cont = False
                    break

                continue

            # if we get too close
            if self.right_scan <= scan_min_threshold:

                rospy.loginfo("Do we break here3????")


                self.tight_left()
                rate.sleep()
                self.go_forward()
                rate.sleep()

                # dist = self.calc_distance(self.current_position.x, self.goal.x, self.current_position.y, self.goal.y)
             
                # if (dist < self.min_distance):
                #     self.obstacle_memory = self.current_position
                #     self.min_distance = dist

                # delta_x = self.current_position.x - self.position_memory.x
                # delta_y = self.current_position.y - self.position_memory.y
                # delta_x = np.absolute(delta_x)
                # delta_y = np.absolute(deltarospy)

                # rospy.loginfo(delta_x)
                # rospy.loginfo(delta_y)

                # if (delta_x < delta_threshold and delta_y < delta_threshold ):
                #     rospy.loginfo("Breaking")
                #     cont = False
                #     break

                continue


        # Turn rover around 180 degrees
        self.turn_around()


        # Circle around the rock going the other direction
        rospy.loginfo("Entering second part of circle algo")
        rate.sleep()

        cont2 = True
        while cont2:

            # General idea:
            # 'Fall through' these
            # while statements
            # which will make rover move
            # in a circular path around obstacle



            # If we turn too far way, hug to the
            # right again
            if self.left_scan > scan_threshold:

                # rospy.loginfo(self.current_position)
                self.tight_left()
                rate.sleep()
                self.go_forward()
                rate.sleep()

                delta_x = self.current_position.x - self.obstacle_memory.x
                delta_y = self.current_position.y - self.obstacle_memory.y
                delta_x = np.absolute(delta_x)
                delta_y = np.absolute(delta_y)

                rospy.loginfo(delta_x)
                rospy.loginfo(delta_y)

                if (delta_x < delta_threshold and delta_y < delta_threshold ):
                    rospy.loginfo("Breaking")
                    cont2 = False
                    break

                continue


            # other wise just keep going forward
            if self.left_scan <= scan_threshold and self.left_scan > scan_min_threshold:

                # rospy.loginfo(self.current_position
                self.go_forward()
                rate.sleep()

                delta_x = self.current_position.x - self.obstacle_memory.x
                delta_y = self.current_position.y - self.obstacle_memory.y
                delta_x = np.absolute(delta_x)
                delta_y = np.absolute(delta_y)
                rospy.loginfo(delta_x)
                rospy.loginfo(delta_y)

                if (delta_x < delta_threshold and delta_y < delta_threshold ):
                    rospy.loginfo("Breaking")
                    cont2 = False
                    break

                continue

            # if we get too close
            if self.left_scan <= scan_min_threshold:

                # rospy.loginfo(self.current_position)
                
                # rospy.loginfo(self.current_position)
                self.tight_right()
                rate.sleep()
                self.go_forward()
                rate.sleep()

                delta_x = self.current_position.x - self.obstacle_memory.x
                delta_y = self.current_position.y - self.obstacle_memory.y
                delta_x = np.absolute(delta_x)
                delta_y = np.absolute(delta_y)
                rospy.loginfo(delta_x)
                rospy.loginfo(delta_y)

                if (delta_x < delta_threshold and delta_y < delta_threshold ):
                    rospy.loginfo("Breaking")
                    cont2 = False
                    break

                continue


       # deprecated, we should let something external handle this
       # self.turn_theta_goal(self.current_position, self.goal)


        rate.sleep()
        rospy.loginfo("Leaving circle algo")
        self.stop()


        return True


    # Sets position we first encounter an obstacle so we know
    # when to stop when we go all the way around the rock
    def set_memory(self):

        self.position_memory = self.current_position

      
         

def main():
    try:
   
        bug1 = Bug1()
        rospy.spin()
    except:
        rospy.loginfo('bug1 node terminated')

if __name__ == "__main__":
    main()