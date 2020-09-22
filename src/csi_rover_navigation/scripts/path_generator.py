"""

PathGenerator script for the CSI NASA 
robotics challenge. 

Used in the navigation package. Integrated
with the global_plannar node. Currently 
there's one path implemented. 

Author: Neil Nie
Craeted: 2020-04-26
Copyright: Columbia Space Initiative

"""

import matplotlib.pyplot as plt
import numpy as np
import cv2 


class PathGenerator:

    def __init__(self):

        # all numbers are in meters
        # all constant definition
        self.w_width = 120   # y
        self.w_length = 140  # x

        self.loop_width = 5
        self.side_pad = 10
        self.linear_step = 15

        self.path_start = (-60, -50)
        # end of constant definition

        waypoints = self.generator_rounded_path()
        # self.plot_path(waypoints)

    def generator_rounded_path(self):

        waypoints = []

        start_x = self.path_start[0]
        start_y = self.path_start[1]

        for y in range(start_y, (self.w_width / 2 - self.side_pad + 1), self.linear_step):

            if start_x < 0:

                # from negative x to positive bound
                for x in range(start_x, (self.w_length / 2 - self.side_pad + 1), self.linear_step):
                    waypoints.append((x, y))
                
                # make sure the turn around point is out of bound
                if (y + self.linear_step * 1/3) <= (self.w_width / 2 - self.side_pad):
                    # turn around point
                    waypoints.append(((self.w_length / 2 - (self.side_pad * 3/4)), 
                                      y + self.linear_step * 1/3))
                    waypoints.append(((self.w_length / 2 - (self.side_pad * 3/4)), 
                                      y + self.linear_step * 2/3))
                    
            else:
            
                # from positive x to negative bound
                for x in range(start_x, -(self.w_length / 2 - self.side_pad) - 1, -self.linear_step):
                    waypoints.append((x, y))
                
                # make sure the turn around point is out of bound
                if (y + self.linear_step * 1/3) <= (self.w_width / 2 - self.side_pad):
                    # turn around point
                    waypoints.append((-(self.w_length / 2 - (self.side_pad * 3/4)), 
                                      y + self.linear_step * 1/3))
                    waypoints.append((-(self.w_length / 2 - (self.side_pad * 3/4)), 
                                      y + self.linear_step * 2/3))

            start_x = int(-1 * start_x)
        
        return waypoints

    def plot_path(self, waypoints):
       
        plt.xlabel("x")
        plt.ylabel("y")
        
        # set the axis ticks
        plt.xticks(np.arange(-self.w_length / 2, self.w_length / 2, self.linear_step))
        plt.yticks(np.arange(-self.w_width / 2, self.w_width / 2, self.linear_step))
        plt.grid()
        
        # plot the dots
        plt.scatter(*zip(*waypoints)) 
        plt.scatter([-70, 70, -70, 70], [-60, 60, 60, -60])
        plt.show()


if __name__ == "__main__":

    PathGenerator()

