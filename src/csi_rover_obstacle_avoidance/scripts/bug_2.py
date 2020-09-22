#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from csi_rover_obstacle_avoidance.srv import ObstacleAvoidance, ObstacleAvoidanceResponse

LINEAR_SPEED = 40
ANGULAR_SPEED = 20

ROSPY_RATE = 50

GOAL = Point(47, -33, 1)

TOLERANCE = 0.5

class Bug2():
    def __init__(self):
        rospy.init_node('bug2')

        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(ROSPY_RATE)

        self.cmd_vel_pub = rospy.Publisher('/scout_1/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scout_1/laser/scan', LaserScan, self.scan_callback)
        self.odometry = rospy.Subscriber('/debugging/gt_odom', Odometry, self.odometry_callback)
        self.rover_name = rospy.get_param('rover_name')
        self.srv = rospy.Service( "/" + self.rover_name + "/bug_2" , ObstacleAvoidance , self.bug2_srv_callback )
        
        self.left_scan = float('inf')
        self.right_scan = float('inf')
        self.center_scan = float('inf')

        rospy.loginfo("bug_2 service is running")
        rospy.sleep(1)


    def shutdown(self):
        rospy.loginfo('Stopping bug2 algorithm...')
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


    def bug2_srv_callback( self , request ):
        # Set goal from service request
        GOAL.x = request.goal_x
        GOAL.y = request.goal_y
        try:
            bug_2_complete = self.bug2()
            success = ObstacleAvoidanceResponse(bug_2_complete, self.current_position.x, self.current_position.y)
            return success
        except:
            failure = ObstacleAvoidanceResponse(False, self.current_position.x, self.current_position.y)
            return failure

    def scan_callback(self, msg):
        num_scans = len(msg.ranges)
        self.left_scan = min(msg.ranges[2*num_scans//3:num_scans])
        self.right_scan = min(msg.ranges[0:num_scans//3])
        self.center_scan = min(msg.ranges[num_scans//3:2*num_scans//3])

    def odometry_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def translation(self, amount):
        move_cmd = Twist()
        move_cmd.linear.x = LINEAR_SPEED if amount >= 0 else -LINEAR_SPEED
        for t in range(int(abs(amount) * ROSPY_RATE)):
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()
        self.cmd_vel_pub.publish(Twist())
    
    def rotation(self, amount):
        move_cmd = Twist()
        move_cmd.linear.x = LINEAR_SPEED if amount >= 0 else -LINEAR_SPEED
        move_cmd.angular.z = ANGULAR_SPEED

        reset_cmd = Twist()
        reset_cmd.angular.z = 0.0000100010001

        for t in range(int(abs(amount) * ROSPY_RATE)):
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()
            self.cmd_vel_pub.publish(reset_cmd)
            self.rate.sleep()
        self.cmd_vel_pub.publish(Twist())
    
        return False
     
    def at_goal(self):
        dist = lambda p1, p2 : math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        return dist(self.current_position, GOAL) <= TOLERANCE
    
    def at_m_line(self, hit_point):
        p1 = hit_point
        p2 = GOAL

        slope = (p2.y - p1.y) / (p2.x - p1.x)
        intercept = p1.y - (slope * p1.x)

        # equation for m_line: 0 = ax + by + c
        # a: slope
        # b: -1
        # c: intercept

        dist = lambda a, b, c, p : abs((a * p.x) + (b * p.y) + c) / math.sqrt(a**2 + b**2)
        # print(dist(slope, -1, intercept, self.current_position))
        return dist(slope, -1, intercept, self.current_position) <= TOLERANCE

    def turn_to_goal(self, hit_orientation):
        dist = lambda p1, p2 : (p1.x * p2.x) + (p1.y * p2.y) + (p1.z * p2.z)

        while dist(self.current_orientation, hit_orientation) < .975:
            self.rotation(0.5)

    def bug2(self):
        while True:
            # if at goal, return
            if self.at_goal():
                rospy.loginfo('bug2: goal reached')
                return True



            # if at obstacle, follow
            elif self.center_scan <= 2:
                rospy.loginfo('bug2: obstacle detected')

                hit_point = self.current_position
                hit_orientation = self.current_orientation

                # turn away from obstacle
                while self.center_scan < 4:
                    self.rotation(0.5)
                
                hit_point_cooldown = 3

                while True:
                    if self.at_goal():
                        break
                    
                    if self.at_m_line(hit_point) and hit_point_cooldown == 0:
                        rospy.loginfo('bug2: m-line reached')
                        self.turn_to_goal(hit_orientation)
                        break


                    # turn away from obstacle
                    while self.right_scan < 4:
                        self.rotation(0.5)
                    
                    # move away from obstacle
                    self.translation(2)
                    
                    # turn back towards obstacle
                    while self.right_scan > 8:
                        self.rotation(-0.5)
                    
                    if hit_point_cooldown > 0:
                        hit_point_cooldown -= 1
            
            # if not at obstacle, move forward
            else:
                self.translation(0.5)

def main():
    try:
        bug2 = Bug2()
        rospy.spin()

    except:
        rospy.loginfo('bug2 node terminated')

if __name__ == "__main__":
    main()