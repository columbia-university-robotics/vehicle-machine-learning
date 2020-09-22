#!/usr/bin/env python
import rospy
import numpy as np
import random
from std_msgs.msg import String

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


# Frontier Based Exploration Algorithim. Based on the "wavefront" flavor 
# of Frontier Algorithims
# Based on the Wavefront paper here:
# Read the original Frontier paper here: 
# Note: We use the original frontier paper for the actual frontier detection as the wavefront
# algo paper does not mention how to do this

# This algorthim determines the location of the nearest frontier to go to.
# It publishes this location (TODO), but does not tell the rover how to get there

# Data structure for points that keeps track of whether
# a point was visited or not
class wave_point:
    def __init__(self, point):

        self.point = point
        self.map_open = False
        self.frontier_open = False
        self.map_closed = False
        self.frontier_closed = False

    def mark_map_open(self):
        self.map_open = True

    def mark_map_closed(self):
        self.map_closed = True

    def mark_frontier_open(self):
        self.frontier_open = True

    def mark_frontier_closed(self):
        self.frontier_closed = True



def callback1(data):
   print("Testing1")


def callback2(data, map_data):
   # build a [widdth] x [height] two dimensional array
   # from the int8[] data passed in. Will need MapMetaData
   print("Testing2")

def frontier():
    # Subscribe to odomety to get current pose (not integrated yet)
    rospy.Subscriber('/scout_1/ekf_odom', Odometry, callback1)
    rospy.Subscriber('/map', OccupancyGrid, callback2)

    # Publish a point of the nearest frontier (not integrated yet)
    # The topic listed does not exsist yet, but the idea is that the wavefront
    # will simply publish WHERE the closet frontier is, but will NOT tell the robot
    # how to get there. Thus it publishes a "goal"
    pub = rospy.Publisher('/scout_1/goal', Point, queue_size=10)


    rospy.init_node('wavefront', anonymous=True)
    rate = rospy.Rate(20) # 20hz    


    # get Meta Data about the occupancy grid
    # m = MapMetaData()
    
    # TODO dummy map for now
    grid_size = 100
    m = np.zeros((grid_size, grid_size))
    
    positions = np.empty((grid_size,grid_size), dtype=object)


    # fill grid with random data for now (TODO Remove)
    for i in range(0, grid_size):
        for j in range(0, grid_size):
            if (random.random() > 0.5):
                m[i][j] = random.random()
               
            else:
                # set to prior prob (see isFrontier below)
                m[i][j] = 0.5
            
            point = Point
            point.x = i
            point.y = j
            point.z = 0 # ignored for now
            positions[i][j] = wave_point(point)


    # the dummy "origin" (TODO Remove)
    # Make this the current pose of the robot instead
    wv = positions[50][50]

    # Call the wavefront algo
    wavefront_algorithim(wv, m, positions)


# returns vector and angle to nearest frontier (No return yet, in development)
def wavefront_algorithim(wv, m, p):
    grid_size = 100

    ogrid_width = grid_size # m.width  
    ogrid_height = grid_size # m.height  

    map_open_list = []
    frontier_open_list = []

    # start algo by adding global pose to the open list
    map_open_list.append(wv)

    # mark as map open list
    wv.mark_map_open()

    while(len(map_open_list) != 0):

        # dequeue first item in the array
        candidate_pt = map_open_list.pop(0)

        # if marked as map close list
        if candidate_pt.map_open == False:
            continue
        
        # if candidate is a frontier point
        if isFrontier(candidate_pt, m):
            print("This is a frontier point ")

            # the frontier we will add to for each continguous frontier
            # point that we find
            new_frontier = []

            frontier_open_list.append(candidate_pt)

            # mark as frontier open list
            candidate_pt.mark_frontier_open()


            while(len(frontier_open_list) != 0):
                 # dequeue first item in the array
                frontier_candidate_pt = frontier_open_list.pop(0)

                # If marked as map clos if ( y_plus_1 < grid_size):
                # if it is a frontier point
                if isFrontier(frontier_candidate_pt, m):
                    print("another frontier pt")
                    new_frontier.append(frontier_candidate_pt)
                    

                    # make sure we fit within the bounds specified

                    x = frontier_candidate_pt.point.x  
                    y = frontier_candidate_pt.point.y  

                    x_plus_1 = x + 1
                    x_minus_1 =  x - 1
                    y_plus_1 = y + 1
                    y_minus_1 = y - 1

                    # Check all the neighboring points:
                    # code looks a little messy, but all the if
                    # statements do is avoid an index out of bounds error
                    if (x_plus_1 < grid_size):
                            if ( y_plus_1 < grid_size):
                                point = p[x+1][y+1]
                                if (check_marks(point)):
                                    frontier_open_list.append(point)
                                  

                            point = p[x + 1][y + 0]
                            if (check_marks(point)):
                                frontier_open_list.append(point)

                        
                            if (y_minus_1 >= 0):
                                point = p[x + 1][y - 1]
                                if (check_marks(point)):
                                    frontier_open_list.append(point)

                    if ( y_plus_1 < grid_size):
                        point = p[x + 0][y + 1] 
                        if (check_marks(point)):
                            frontier_open_list.append(point)
                        
                        if (x_minus_1 >= 0):
                            point = p[x - 1][y + 1]
                            if (check_marks(point)):
                                    frontier_open_list.append(point)
                                
                    
                    if (y_minus_1 >= 0):
                            
                        point = p[x + 0][y - 1]
                        if (check_marks(point)):
                            frontier_open_list.append(point)
                            
                            
                        if (x_minus_1 >= 0):
                            point = p[x - 1][y - 1]
                            if (check_marks(point)):
                                frontier_open_list.append(point)
                            
                              

                    if (x_minus_1 >= 0):
                        point = p[x - 1][y + 0]
                        if (check_marks(point)):
                                frontier_open_list.append(point)


                    # mark frontier point as closed
                    frontier_candidate_pt.mark_frontier_closed()
                # We need to save the new_frontier data... serialization? open queston
                # save(new_frontier) # no function exsists yet

                # mark everything in the new frontier list as map close list
               
                for i in new_frontier:
                    i.mark_map_closed()


        # for all the adjacent points

        # make sure we fit within the bounds specified

        x = candidate_pt.point.x  
        y = candidate_pt.point.y  

        x_plus_1 = x + 1
        x_minus_1 =  x - 1
        y_plus_1 = y + 1
        y_minus_1 = y - 1

        # Check all the neighboring points:
        # code looks a little messy, but all the if
        # statements do is avoid an index out of bounds error
        if (x_plus_1 < grid_size):
                if ( y_plus_1 < grid_size):
                    point = p[x+1][y+1]
                    if (check_outer_marks(point)):
                        map_open_list.append(point)
                        

                point = p[x + 1][y + 0]
                if (check_outer_marks(point)):
                    map_open_list.append(point)

            
                if (y_minus_1 >= 0):
                    point = p[x + 1][y - 1]
                    if (check_outer_marks(point)):
                        map_open_list.append(point)

        if ( y_plus_1 < grid_size):
            point = p[x + 0][y + 1] 
            if (check_outer_marks(point)):
                map_open_list.append(point)
            
            if (x_minus_1 >= 0):
                point = p[x - 1][y + 1]
                if (check_outer_marks(point)):
                        map_open_list.append(point)
                    
        
        if (y_minus_1 >= 0):
                
            point = p[x + 0][y - 1]
            if (check_outer_marks(point)):
                map_open_list.append(point)
                
                
            if (x_minus_1 >= 0):
                point = p[x - 1][y - 1]
                if (check_outer_marks(point)):
                    map_open_list.append(point)
                
                    

        if (x_minus_1 >= 0):
            point = p[x - 1][y + 0]
            if (check_outer_marks(point)):
                   map_open_list.append(point)
                

        # mark as map closed list
        candidate_pt.mark_map_closed()

    rospy.loginfo("All done :)")


def check_marks(point):

    if (point.frontier_open == False and point.map_closed == False and point.frontier_closed == False):
        point.mark_frontier_open()
        return True
    else:
        return False


def check_outer_marks(point):

    if (point.map_open == False and point.map_closed == False):

        # WIP: Check for one map open neighbor for each point.
        # Checking for one map open neighbor is what makes this algorithim
        # a wavefront algortithm. This can be considered a "defining" moment
        # despite how mundane it appears

        if (hasMapOpenNeighbor(point)):
            print("Success!")
            point.mark_map_open()
            return True
        else:
            return False


       
    else:
        return False

# checks neighboring vertices to see
# if one is marked as "map open"
def hasMapOpenNeighbor(point):

    grid_size = 100
    # Poor notaton, but all this is doing
    # is getting the position coordinates
    x = point.point.x  
    y = point.point.y  

    x_plus_1 = x + 1
    x_minus_1 =  x - 1
    y_plus_1 = y + 1
    y_minus_1 = y - 1

    # Check all the neighboring points:
    # code looks a little messy, but all the if
    # statements do is avoid an index out of bounds error
    if (x_plus_1 < grid_size):
            if ( y_plus_1 < grid_size):
                point = p[x+1][y+1]
                if(point.map_open):
                    return True
                    

            point = p[x + 1][y + 0]
            if (point.map_open):
                return True

        
            if (y_minus_1 >= 0):
                point = p[x + 1][y - 1]
                if (point.map_open):
                    return True

    if ( y_plus_1 < grid_size):
        point = p[x + 0][y + 1] 
        if (point.map_open):
            return True
        
        if (x_minus_1 >= 0):
            point = p[x - 1][y + 1]
            if (point.map_open):
                    return True
                
    
    if (y_minus_1 >= 0):
            
        point = p[x + 0][y - 1]
        if (point.map_open):
            return True
            
            
        if (x_minus_1 >= 0):
            point = p[x - 1][y - 1]
            if (point.map_open):
                return True
                     

    if (x_minus_1 >= 0):
        point = p[x - 1][y + 0]
        if (point.map_open):
                return True


    # If we get to this point, there are no map-open neighbors
    # so we return false
    else:
        return False

# frontier detection based on Yamauchi's original paper
def isFrontier(candidate_pt, m):
    # Key observation: Any open cell adjacent to an unknown 
    # cell is labeled a frontier edge cell

    # this is a hyper parameter for the "occupied" threshold
    # 0.5 is the value suggested by Yamauchi
    grid_size = 100
    prior_prob = 0.5

    # x and y coords of candidate point
    x = candidate_pt.point.x
    y = candidate_pt.point.y


    # make sure we fit within the bounds specified
    x_plus_1 = x + 1
    x_minus_1 =  x - 1
    y_plus_1 = y + 1
    y_minus_1 = y - 1

    # Check all the neighboring points:

  
    if (x_plus_1 < grid_size):
        if ( y_plus_1 < grid_size):
            if (m[x + 1][y + 1] == 0.5):
                return True
        if (m[x + 1][y + 0] == 0.5):
            return True   

        if (y_minus_1 >= 0):
            if (m[x + 1][y - 1] == 0.5):
                return True

    if ( y_plus_1 < grid_size):
        if (m[x + 0][y + 1] == 0.5):
            return True
       
        if ( x_minus_1 >= 0):
            if (m[x - 1][y + 1] == 0.5):
                return True  
   
    if (y_minus_1 >= 0):
    
        if (m[x + 0][y - 1] == 0.5):
            return True
        
        if (x_minus_1 >= 0):
            if (m[x - 1][y - 1] == 0.5):
                return True

    if (x_minus_1 >= 0):
        if (m[x - 1][y + 0] == 0.5):
            return True
  

    # if we reach here, our candidate point is not a frontier pt
    else:
        return False




if __name__ == '__main__':
    try:
        frontier()
    except rospy.ROSInterruptException:
        pass
