#!/usr/bin/env python
#
############################
#  Bug Tangent Algorithm   #
#  Author: Nathalie Hager  #
#  Uni: nmh2147            #
############################

import rospy
import numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class BugTan():
    def __init__(self):
        rospy.init_node('bugtan')

        rospy.on_shutdown(self.shutdown)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scout_1/laser/scan', LaserScan, self.scan_callback)

        self.left_scan = float('inf')
        self.right_scan = float('inf')
        self.center_scan = float('inf')

        self.current_position = Point(0,0,0)
        self.goal = Point(0,0,0)
        self.set_Oi = []
        self.direct_path_blocked = False

    def shutdown(self):
        rospy.loginfo('Stopping bug tan algorithm...')
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def scan_callback(self, msg):

        """ #1 BUILD SET {O_i} (the set of endpoints) """
        self.set_Oi = []

        i = 0
        while i < len(msg.ranges):
            # check if obstacle begins (if so, we get a distance value in the scanner's range)
            if ranges[i] in range(msg.range_min, msg.range_max):

                # obstacle detected: start finding the two points of discontinuity of the perceived obstacle

                # save 'start' point of obstacle (as a global coordinate)
                self.set_Oi.append(find_coordinate(msg.ranges[i], i, msg.angle_min, msg.angle_increment))

                # now go through this interval of continuity until obstacle ends
                while ranges[i] in range(msg.range_min, msg.range_max):
                    if i < len(msg.ranges) - 1:
                        i += 1

                # go back to the last ranges[...] value that was within range and hence is the endpoint of the perceived obstacle
                i = i - 1

                # save 'end' point of obstacle (as a global coordinate)
                self.set_Oi.append(find_coordinate(msg.ranges[i], i, msg.angle_min, msg.angle_increment))

                # proceed with the ranges[...] value right after the endpoint of the perceived obstacle
                i += 1
            else:
                # no obstacle detected at this ranges[...] value
                # proceed with the next value
                i += 1

        """ #2 CHECK IF ANY OBSTACLE BLOCKS DIRECT PATH TO GOAL FROM CURRENT POSITION """
        # circle chord connecting the two outmost points we have to scan in order
        # to evaluate whether the rover safely can drive through the area right
        # in front of it
        # (the circle is describing the max. range of the 2D lidar scanner
        # -> imagine drawing the scanner range around the rover on a piece of paper
        # to understand what (partial) circle I mean)
        chord = 2.3 # rounded up (rover width: 2.2098 meters)
        radius = msg.radius

        # find scan angle range that we need to check for obstacles
        half_angle = numpy.arcsin((chord/2) / radius)
        bins = numpy.ceil(half_angle / msg.angle_increment)   # we need to check these bins for obstacle range measurements in msg.ranges[...]

        obstacle_detected = False
        i = -1 * bins
        while i <= bins:
            if msg.ranges[(len(msg.ranges) / 2) + i] in range(msg.range_min, msg.range_max):
                # obstacle detected
                obstacle_detected = True
                break

            i += 1

        self.direct_path_blocked = obstacle_detected


    # returns the coordinate as a 2D Point() object from a given angle and distance given by the laser scanner
    def find_coordinate(self, dist, i, angle_min, angle_increment):

        # get polar coordinate
        angle = angle_min + (i * angle_increment)
        r = dist

        # get cartesian coordinate
        local_x = r * numpy.cos(angle)
        local_y = r * numpy.sin(angle)

        # since the above coordinates take the rover's location as the origin,
        # we need to adjust them to work with the global coordinate system
        global_coordinate = Point()
        global_coordinate.x = self.current_position.x + local_x
        global_coordinate.y = self.current_position.x + local_y

        return global_coordinate


    def odometry_callback(self, msg):
        self.current_position = msg.pose.pose.position

    # runs the bugtan algorithm until the goal has been reached or it has been determined that the goal can't be reached
    def run_bug(self, goal):

        # declare point n
        # start by moving toward goal: choose point n = goal
        self.goal = goal

        # set point n
        self.local_goal = ?? #select O from set O_i which minimizes the heuristic distance

        while True:
            #rospy.loginfo('{}, {}, {}'.format(self.left_scan, self.center_scan, self.right_scan))
            rate = rospy.Rate(1) # 10hz

            if goal_encountered():
                # if goal has been encountered, stop rover
                # report success: goal has been reached
                self.cmd_vel_pub.publish(Twist())
                rospy.loginfo("reached goal (success)")
                rospy.sleep(1)
                return

            # boundary following behaviour
            # if local minimum has been detected
            elif local_minimum_detected():

                # choose a boundary following direction which continues in the same direction as the most recent motion-to-goal direction.

                while True:
                    if goal_encountered():
                        # stop rover
                        # report success: goal has been reached
                        self.cmd_vel_pub.publish(Twist())
                        rospy.loginfo("reached goal (success)")
                        rospy.sleep(1)
                        pass

                    elif cycle_completed():
                        # stop rover
                        # report failure: goal cannot be reached
                        pass

                    elif d_leave < d_min:
                        # terminate the boundary-following behavior
                        # set new point n towards which the rover will drive
                        # clear all local minimum variables
                        # break out of this while loop
                        pass

                    else:
                        # Continuously update d_leave, d_min, and {O_i}.
                        # Continuously moves toward n ∈ {O_i} that is in the chosen boundary direction.
                        pass

            # motion to goal behaviour
            else:
                rospy.loginfo('set of points of discontinuity {O_i}:')
                rospy.loginfo(self.set_Oi)

                # if there is an obstacle between the robot and the goal
                if self.direct_path_blocked:
                    # find point n ∈ {T,O_i} which minimizes d(x, n) + d(n, q_goal) where d(n, q_goal) < d(x, q_goal)
                    min_dist = 1000000
                    min_point = Point()    # min_point == point n which minimizes d(x, n) + d(n, q_goal)

                    for n in self.set_Oi:
                        x = self.current_position

                        if calc_distance(n, self.goal) < calc_distance(x, self.goal):
                            current_dist = calc_distance(x, n) + calc_distance(n, self.goal)    # maybe later a better heuristic can be used here
                            if current_dist < min_dist:
                                min_dist = current_dist
                                min_point = n

                    # continuously move toward the minimizing point
                    move_toward(min_point)

                else:
                    move_toward(self.goal)

    # checks if the rover has encountered the goal yet
    def goal_encountered(self):
        tolerance = 1

        if calc_distance(self.current_position, self.goal) <= tolerance:
                return True
        return False

    # calculates the Euclidean distance between two points (2 dimensional)
    # source: bug_1.py from Mikey
    def calc_distance(self, current_pos, goal_pos):
        x_pos = current_pos.x
        y_pos = current_pos.y
        x_goal = goal_pos.x
        y_goal = goal_pos.y

        delta_goal_x = x_pos - x_goal
        delta_goal_y = y_pos - y_goal
        delta_x_square = delta_goal_x ** 2
        delta_y_square = delta_goal_y ** 2

        dist_square = delta_x_square + delta_y_square

        dist = math.sqrt(dist_square)
        rospy.loginfo(dist)
        return dist

    # checks if local minimum has been reached when trying to drive around the current obstacle
    def local_minimum_detected(self):
        pass

    # checks if rover has completed a cycle (i.e. if it drove once completely around the obstacle)
    def cycle_completed(self):
        pass

    # checks if there is an obstacle blocking the direct path from the current position to the goal
    def obstacle_blocking(self):
        pass

        # moves towards a given 2D target coordinate
    def move_toward(self, target):
        self.rover_name = rospy.get_param('rover_name')
        self.service_name = "/" + self.rover_name + "/target_coordinate"

        rospy.wait_for_service( self.service_name )
        rospy.loginfo_throttle(2, "current_target : " + str( self.target ) )

        try:
            target_service = rospy.ServiceProxy( self.service_name , TargetCoordinate)

            # we can also use Header() but you'd have to input the frame_id so I figured "client_name" was more direct.
			response = target_service(  x = self.target[0], y = self.target[1], client_name = "SCRIPT_NODE_NAME")

			if( response.arrived ):
                rospy.loginfo_throttle(2, "Yay rover arrived !" )

        except rospy.ServiceException, e:
            print( "Service call failed: %s"%e )
            rospy.wait_for_service( self.service_name )

        rate.sleep()


    def test_bug(self):
        # test goal_encountered()
        rospy.loginfo("goal_encountered: " + goal_encountered())

        # test if scan_callback() finds the set {O_i} correctly
        rospy.loginfo("set {O_i}: " + set_Oi)

        # test if move_toward works as expected
        # IMPLEMENT THIS

        rospy.sleep(1)

def main():
    try:
        bugtan = BugTan()
        #bugtan.run_bug()
        bugtan.test_bug()

    except:
        rospy.loginfo('bugtan node terminated')

if __name__ == "__main__":
    main()
