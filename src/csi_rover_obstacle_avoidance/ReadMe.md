# Obstacle Avoidance ReadMe
All obstacle avoidance will be run as a service. There is not obstacle detection within the logic of any of the algos.
To run an Obstace Avoidance Service, do the following:

### Service Server Setup:

rosrun csi_rover_obstacle_avoidance bug_[#].py 

example: rosrun csi_rover_obstacle_avoidance bug_1.py 


### Service Client Setup

rosservice call /scout_1/bug_[#] -- [goal_x goal_y]

example: rosservice call /scout_1/bug_1 -- 28 -15

NOTE: The obstacle avoidance will not reach this goal. It will use it as a waypoint to align itself as it
moves around the obstacle.

### Service Client Return

All obstacle Avoidance will return a boolean of whether it is successful.
It will also return its end position. This return structure can be found in the srv folder.

Example output: 

arrived: False
end_x: -40.5515283704
end_y: -43.4767090027


## Administrative Section
The following section will be updated to track the progress of Obstacle Avoidance work

### Done:

1. Bug1
2. Bug2
3. Bug as a service

### TODO

1. Bug0
2. BagTan
3. Simple Avoidance
4. Update from cmd vel avoidance
