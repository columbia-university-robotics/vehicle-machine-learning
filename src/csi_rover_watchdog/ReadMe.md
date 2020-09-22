## Watchdog ReadMe

The basic concept of the watch dog node is to monitor both the system, and the simulation. More importanly,
this node will take action when it detects anomalies in either the sytem or the simulation, by throwing errors, restarting nodes,
or even reseting the Rover in the simulation.


### Running the Watchdog

To run the watchdog node type: rosrun csi_rover_watchdog watchdog.py

From there, it will work in the background. It will print error messages when it is forced to reset the Rover.


### System

1. [TODO]: if any of our nodes unexpectedly dies, we need to respawn or let someone know. (some nodes are already respawn automatically)
2. [TODO]: if NASA’s simulation stops sending us data via their topics, we need to throw an error. 

### Simulation

1. [IN TESTNG]: the watchdog should monitor the position of the rover. if it collides and gets stuck, call reset. 
2. [IN TESTNG]: if the rover flips over, call reset. 





