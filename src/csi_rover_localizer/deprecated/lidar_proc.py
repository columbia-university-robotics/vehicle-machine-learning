#!/usr/bin/env python
# Converts laser scan to planar distance
# and segments into obstalces and just hills

import rospy
import sys
from std_msgs.msg import Header , Bool , Float64
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy
import math

import scipy.stats
import numpy as np
import json

# Map class is largely built off of code at https://gist.github.com/superjax/33151f018407244cb61402e094099c1d
class Map():
    def __init__(self, xsize, ysize, grid_size):
        self.xsize = xsize+2 # Add extra cells for the borders
        self.ysize = ysize+2
        self.grid_size = grid_size # save this off for future use
        self.log_prob_map = np.zeros((self.xsize, self.ysize)) # set all to zero

        self.alpha = 1.0 # The assumed thickness of obstacles
        self.beta = 5.0*np.pi/180.0 # The assumed width of the laser beam
        self.z_max = 15.0 # The max reading from the laser

        # Pre-allocate the x and y positions of all grid positions into a 3D tensor
        # (pre-allocation = faster)
        self.grid_position_m = np.array([np.tile(np.arange(0, self.xsize*self.grid_size, self.grid_size)[:,None], (1, self.ysize)),
                                         np.tile(np.arange(0, self.ysize*self.grid_size, self.grid_size)[:,None].T, (self.xsize, 1))])

        # Log-Probabilities to add or remove from the map 
        self.l_occ = np.log(0.65/0.35)
        self.l_free = np.log(0.35/0.65)

    def update_map(self, pose, z):

        dx = self.grid_position_m.copy() # A tensor of coordinates of all cells
        dx[0, :, :] -= pose[0] # A matrix of all the x coordinates of the cell
        dx[1, :, :] -= pose[1] # A matrix of all the y coordinates of the cell
        theta_to_grid = np.arctan2(dx[1, :, :], dx[0, :, :]) - pose[2] # matrix of all bearings from robot to cell

        # Wrap to +pi / - pi
        theta_to_grid[theta_to_grid > np.pi] -= 2. * np.pi
        theta_to_grid[theta_to_grid < -np.pi] += 2. * np.pi

        dist_to_grid = scipy.linalg.norm(dx, axis=0) # matrix of L2 distance to all cells from robot

        # For each laser beam
        for z_i in z:
            r = z_i[0] # range measured
            b = z_i[1] # bearing measured

            # Calculate which cells are measured free or occupied, so we know which cells to update
            # Doing it this way is like a billion times faster than looping through each cell (because vectorized numpy is the only way to numpy)
            free_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (dist_to_grid < (r - self.alpha/2.0))
            occ_mask = (np.abs(theta_to_grid - b) <= self.beta/2.0) & (np.abs(dist_to_grid - r) <= self.alpha/2.0)

            # Adjust the cells appropriately
            self.log_prob_map[occ_mask] += self.l_occ
            self.log_prob_map[free_mask] += self.l_free

class DepthImageProc:
    ''' Takes in stereo camera depth image and produces estimates of detected
    obstacles
    '''
    def __init__(self):
        self.bridge = CvBridge()
        # TODO: UPDATE WITH ACTUAL FIELD OF VIEW
        self.fov = 100 # field of view in degrees
        self.NUM_BUCKETS = 100
        pub_topic = rospy.get_param('~vision_laser_scan', '/vision/laser/scan')
        self.laser_pub = rospy.Publisher(pub_topic, LaserScan, queue_size=4)

    def hook_in(self, depth_img_topic, range_to_obstacle):
        self.range_to_obstacle = range_to_obstacle
        rospy.Subscriber(depth_img_topic, Image, callback=self.depth_callback)

    def _to_rad(self, deg):
        return deg*(3.14159265/180)

    def _idx_to_degree(self, idx, length):
        # TODO: CORRECT THIS CODE
        # TRIGONOMETRY IS A BIT WONKY
        rad_fov = self._to_rad(self.fov)
        # computinng hypothetical bisector of
        # isosceles triangle with unneven
        # length of `length`
        jut_length = length*math.tan(rad_fov)
        if idx > length/2:
            return math.atan((idx - length/2)/jut_length)*(180/3.14159265)
        else:
            return -math.atan((length/2 - idx)/jut_length)*(180/3.14159265)
        '''
        centered_idx = idx - length/2.0
        degrees_per_idx = self.fov/length
        return centered_idx * degrees_per_idx
        '''

    def _deg_to_bucket(self, deg, num_buckets):
        # TODO: VALIDATE
        deg_per_bucket = self.fov/num_buckets
        bucket = int((deg + self.fov/2) / deg_per_bucket)
        return bucket

    def depth_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "32FC1")
        depths = np.array(img, dtype = np.float32)
        width = len(depths[0])
        buckets = [[] for i in range(self.NUM_BUCKETS)]
        for row in depths:
            for i, val in enumerate(row):
                if not np.isnan(val):
                    deg = self._idx_to_degree(i, len(depths[0]))
                    idx = self._deg_to_bucket(deg, self.NUM_BUCKETS)
                    buckets[idx].append(val)
        # conform to right hand rule direction
        buckets.reverse()
        # compute candidate dist from observed distances
        median_buckets = [np.median(i) if len(i) > 0 else float('inf') for i in buckets]
        # min_buckets = [np.min(i) if len(i) > 0 else float('inf') for i in buckets]
        #print('width', width)
        #print('buckets', median_buckets)
        obstacles = self.range_to_obstacle(median_buckets, self.fov/self.NUM_BUCKETS,
                -self._to_rad(self.fov/2))
        print(obstacles)
        # print([sum([float(i) for i in j if not i == np.nan]) for j in depths])

        # now publish
        # TODO: MAKE PUBLISH FROM ONLY IDENTIFIED OBSTACLE POINTS
        self.publish_laser_scan(median_buckets, data)

    
    def publish_laser_scan(self, ranges, depth_msg):
        new_msg = LaserScan()
        # TODO: Add timing information
        new_msg.header = depth_msg.header
        new_msg.angle_min = -self._to_rad(self.fov/2)
        new_msg.angle_max = self._to_rad(self.fov/2)
        new_msg.angle_increment = self._to_rad(self.fov) / self.NUM_BUCKETS
        new_msg.time_increment = 0.0
        new_msg.range_min = 0.0
        new_msg.range_max = float('inf')
        new_msg.ranges = ranges
        new_msg.intensities = [0.0]*len(ranges)
        #print(new_msg)
        self.laser_pub.publish(new_msg)

class LidarProc:
    ''' Takes in 2D lidar data, generates useful values out of it and then
        detects obstacles and builds an occupancy grid.
    '''
    OCCUPANCY_UPDATE_RATE = 5 # update every 10 LaserScan frames

    def __init__(self):
        self.orientation = None
        self.pose = None
        self.messages_since_update = self.OCCUPANCY_UPDATE_RATE
        rospy.init_node('lidar_proc')

        self.debug = rospy.get_param('~debug', False)

        self.enable_occupancy = rospy.get_param('~enable_occupancy', True)
        self.print_collision = rospy.get_param('~print_collision', False)

        self.buffer_distance = rospy.get_param('~buffer_distance', True)

        self.rover_name = rospy.get_param('~rover_name', 'scout_1')
        self.rover_width = rospy.get_param('/rover_total_width', 2.2098)

        imu_topic = rospy.get_param('~imu_topic', self.rover_name + '/imu')
        rospy.Subscriber(imu_topic, Imu, callback=self.imu_callback)
        odom_topic = rospy.get_param('~odom_topic', '/' + self.rover_name + '/ekf_odom')
        rospy.Subscriber(odom_topic, Odometry, callback=self.odom_callback)
        laser_scan_topic = rospy.get_param('~laser_scan_topic', self.rover_name + '/laser/scan')
        rospy.Subscriber(laser_scan_topic, LaserScan, callback=self.laser_scan_callback)
        
        self.laser_pub = rospy.Publisher("/" + self.rover_name + '/laser/filtered', LaserScan, queue_size=10)
        
        occupancy_topic = rospy.get_param('occupancy_topic', '/lidar_occupancy')
        self.occupancy_pub = rospy.Publisher(occupancy_topic, OccupancyGrid, queue_size=4)

        obstacle_detected_topic = rospy.get_param('obstacle_detected_topic', '/obstacle_detected')
        self.obstacle_detected_pub = rospy.Publisher('/' + self.rover_name + obstacle_detected_topic, Bool, queue_size=1)

        avoid_obstacle_angle_topic = rospy.get_param('obstacle_avoidance_angle_topic', '/avoid_obstacle_angle')
        self.avoid_obstacle_angle_pub = rospy.Publisher('/' + self.rover_name + avoid_obstacle_angle_topic, Float64, queue_size=1)
        avoid_obstacle_dist_topic = rospy.get_param('obstacle_avoidance_dist_topic', '/avoid_obstacle_distance')
        self.avoid_obstacle_dist_pub = rospy.Publisher('/' + self.rover_name + avoid_obstacle_angle_topic, Float64, queue_size=1)
        # self.obstacle_pub = rospy.Publisher('/local_obstacles', LaserScan, queue_size=4)
        
        # Set up Depth Image processor
        myDepthProc = DepthImageProc()
        self.depth_topic = rospy.get_param('~depth_image_topic', '/stereo/depth_image')
        myDepthProc.hook_in(self.depth_topic, self._range_to_obstacle)


        # will be dynamically updated
        self.angle_increment = 0.02626
        self.angle_min = -1.3
        self.angle_max = 1.3

        self.grid_size = 1.0 # 1 meter grid
        self.width = int(140.0/self.grid_size)
        self.height = int(120.0/self.grid_size)
        if self.enable_occupancy:
            self.obstacle_map = Map(self.width, self.height, self.grid_size)
        
        # now set up occupancy grid publishing
        if self.enable_occupancy:
            r = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.pub_occupancy()
                r.sleep()
        else:
            rospy.spin()

    def imu_callback(self, data):
        orien = data.orientation
        quaternion_vals = [orien.x, orien.y, orien.z, orien.w]
        self.orientation = euler_from_quaternion(quaternion_vals)

    def odom_callback(self, data):
        ros_pose = data.pose.pose
        orien = ros_pose.orientation
        quaternion_vals = [orien.x, orien.y, orien.z, orien.w]
        bearing = euler_from_quaternion(quaternion_vals)[2]
        self.pose = [ros_pose.position.x, ros_pose.position.y, bearing]

    def _ranges_to_planar(self, ranges):
        ''' Converts ranges from LaserScan to their planar distance
        '''
        # TODO: Do conversion
        return ranges
    
    def _range_to_obstacle(self, ranges, angle_increment=None, angle_min=None):
        ''' Takes in raw range data and then determines
        what obstacles are there and filters the ranges to
        just the laser hits that belong to an obstacle.
        '''
        # handle implicit parameters
        if angle_increment == None:
            angle_increment = self.angle_increment
        if angle_min == None:
            angle_min = self.angle_min

        JUMP_THRESH = 2 # in meters
        MAX_WIDTH = 5 # 8 # widest boulder is 8 meters
        if self.debug:
            rospy.loginfo('$$$$$$$$$$$$$$$$$$$$$$$$$\n\n')
        candidate_sections = []
        last_depth = float('inf')
        # first find all candidate sections where a jump in range occurs
        curr_sect = []
        for i, range in enumerate(ranges):
            diff = abs(range - last_depth)
            if diff > JUMP_THRESH:
                if len(curr_sect) > 0:
                    candidate_sections.append(curr_sect)
                    curr_sect = []
                if not range == float('inf'):
                    curr_sect.append((i, range))
            else:
                if (len(curr_sect) > 0):
                    curr_sect.append((i, range))
            last_depth = range
        if len(curr_sect) > 0:
            candidate_sections.append(curr_sect)
        if self.debug:
            rospy.loginfo('# sections: %s', len(candidate_sections))
        
        # now differentiate hills from boulders
        confirmed_obstacles = []
        for sect in candidate_sections:
            if len(sect) < 3: # need 3 or more lidar hits
                continue
            cut_ends = (sect[0], sect[-1])
            ends_diff = sect[-1][1] - sect[0][1]
            
            cut_delta = ends_diff / (sect[-1][0] - sect[0][0])
            
            # Brace for trigonometry needed to compute what section would
            # look like if it was a flat wall (to then compute relative convexity)
            # width in radians
            section_width = (sect[-1][0] - sect[0][0])*angle_increment
            # by law of cosines
            right_side = sect[-1][1]
            left_side = sect[0][1]
            far_length = math.sqrt(right_side**2 + left_side**2 - \
                2*right_side*left_side*math.cos(section_width))
            # now find left corner angle using law of cosines
            denominator = -2 * far_length * left_side
            if denominator == 0:
                denominator = 0.1**6
            corner_angle = math.acos((right_side**2 - left_side**2 - far_length**2) /
                denominator)
            avg_dist = 0

            for val in sect[1:len(sect)-1]:
                range = val[1]
                ticks_over = val[0] - cut_ends[0][0]
                cut_dist = cut_ends[0][1] + cut_delta*ticks_over
                # compute using law of cosines the distance if section was
                # a flat wall
                section_angle = ticks_over*angle_increment
                right_top_corner_angle = 2*math.pi - corner_angle - section_angle
                cut_dist = -(left_side*math.sin(corner_angle))/math.sin(right_top_corner_angle)
                avg_dist += (range - cut_dist) / len(sect)
            if self.debug:
                rospy.loginfo('avg_dist %s num_entries %s width %s', avg_dist, len(sect), far_length)
            
            # now check if obstacle or feature of ground plane
            CURVATURE_CUTOFF = 0.2 # average depth must be less than 0.2 (i.e. convex within the lidar error)
            if avg_dist <= CURVATURE_CUTOFF and far_length <= MAX_WIDTH:
                confirmed_obstacles.append(sect)
                obstacle_angle = (cut_ends[0][0]*angle_increment + angle_min, \
                            cut_ends[-1][0]*angle_increment + angle_min)
                rad_to_deg = (360/(2*math.pi))
                obstacle_degrees = (obstacle_angle[0]*rad_to_deg, \
                                    obstacle_angle[1]*rad_to_deg)
                if self.debug:
                    rospy.logwarn('FOUND OBSTACLE BETWEEN %s', obstacle_degrees)
                    rospy.logwarn('AT DIST %s', (cut_ends[0][1] + cut_ends[1][1])/2)

                # now check danger level of obstacle
                # first find collision angle
                # this is just trigonemtry using
                # the rover width and the distance to the obstacle
                # angle = tan^-1(opposite/adjacent)
                o = self.rover_width/2
                a = (cut_ends[0][1] + cut_ends[1][1])/2 + sys.float_info.min
                angle_offset = math.atan(o/a)
                danger_angle = (-angle_offset,angle_offset)
                # now detect if collision course
                boulder_within_danger = (obstacle_degrees[0] >= danger_angle[0] and obstacle_degrees[0] <= danger_angle[1]) or \
                        (obstacle_degrees[1] >= danger_angle[0] and obstacle_degrees[1] <= danger_angle[1])
                danger_within_boulder = (danger_angle[0] >= obstacle_degrees[0] and danger_angle[0] <= obstacle_degrees[1]) or \
                        (danger_angle[1] >= obstacle_degrees[0] and danger_angle[1] <= obstacle_degrees[1])
                if boulder_within_danger or danger_within_boulder:
                    if self.print_collision:
                        rospy.logwarn('!!!COLLISION COURSE!!! AT DIST %s', a)
                    use_right_side = (abs(obstacle_degrees[0]) < abs(obstacle_degrees[1])) or obstacle_degrees[1] < 0
                    # decompose to forward and left/right displacement needed
                    # to avoid obstacle
                    if use_right_side:
                        safe_forward = right_side*math.cos(obstacle_angle[0])
                        safe_right = right_side*math.sin(obstacle_angle[0])+self.buffer_distance
                    else:
                        safe_forward = left_side*math.cos(obstacle_angle[1])
                        safe_right = -left_side*math.sin(obstacle_angle[1])-self.buffer_distance
                    if self.print_collision:
                        rospy.logwarn('!!!AVOIDANCE RELATIVE TARGET: %s, %s!!!', safe_forward, safe_right)
                    self.obstacle_detected_pub.publish(True)
                    self.avoid_obstacle_angle_pub.publish(safe_right)
                    self.avoid_obstacle_dist_pub.publish(a)
                else:
                    self.obstacle_detected_pub.publish(False)
                    self.avoid_obstacle_angle_pub.publish(0)
                    self.avoid_obstacle_dist_pub.publish(0)
        return confirmed_obstacles


    def laser_scan_callback(self, laser_scan):
        self.angle_increment = laser_scan.angle_increment
        self.angle_min = laser_scan.angle_min
        self.angle_max = laser_scan.angle_max
        # Now process range data
        ranges = laser_scan.ranges
        planar_range = self._ranges_to_planar(ranges)
        if not planar_range == None:
            new_scan = deepcopy(laser_scan)
            new_scan.ranges = planar_range
            # now process for obstacles
            obstacles = self._range_to_obstacle(planar_range)
        hits = [] # all the obstacle rays
        for obs in obstacles:
            for measure in obs:
                bearing = self.angle_min + self.angle_increment*measure[1]
                hits.append([measure[0], bearing])
        if self.pose == None:
            rospy.logwarn('For 2D lidar processor, no pose determined.')
        if self.enable_occupancy and self.messages_since_update >= self.OCCUPANCY_UPDATE_RATE and \
             not self.pose == None and len(hits) > 0:
            self.obstacle_map.update_map(self.pose, hits)
            self.messages_since_update = 0
        # publish filtered laser scan
        self.pub_filtered_laser(laser_scan, obstacles) 

        if self.debug and self.enable_occupancy:
            # save occupancy grid for analysis and graphing
            open('laser_obstacles.json', 'w').write(json.dumps([list(i) for i in self.obstacle_map.log_prob_map]))
        
        self.messages_since_update += 1

    def pub_occupancy(self):
        occupancy = []
        for row in self.obstacle_map.log_prob_map:
            for i in row:
                try:
                    occupancy.append(math.exp(i)*100)
                except OverflowError:
                    rospy.logwarn('OverflowError: %s', i)
                    occupancy.append(100)
        msg = OccupancyGrid()
        msg.info.resolution = self.grid_size
        msg.info.width = self.width
        msg.info.height = self.height
        msg.data = occupancy
        self.occupancy_pub.publish(msg)
    
    def pub_filtered_laser(self, og_laser, obstacles):
        ''' Accepts original laser scan message and
        tuple of index and range for scans (all other angles)
        will be set to zero
        '''
        new_msg = deepcopy(og_laser)
        new_msg.ranges = [float('inf')]*len(new_msg.ranges)
        for obs in obstacles:
            for range in obs:
                new_msg.ranges[range[0]] = range[1]
        self.laser_pub.publish(new_msg)

if __name__ == "__main__":
    LidarProc()
