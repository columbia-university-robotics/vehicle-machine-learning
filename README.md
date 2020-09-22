# CSI Rover

For detailed information about the rover model, please visit this [README](src/csi_rover_gazebo/README.md). 

## Setup

Make sure you have ROS melodic and all of the neccesary dependencies installed. 

- First clone the repo: 
`git clone https://github.com/CSIRover/SRC-Phase2.git`
- Then, `cd SRC-Phase2`
- Run `cakin_make`
- Remeber to `source devel/setup.bash`

You should be all set. Please open an issue if there's a problem with the setup process. 

### Launch rover model

1. Navigate to your catkin workspace. 
2. Run `catkin_make`
3. `source devel/setup.bash`
4. `roslaunch csi_rover_gazebo csi_rover_rviz.launch` This is launch with rviz. 


## Packages

Overview of all the packages and their role in the system.

* `csi_rover_controls`: For description please visit [README](src/csi_rover_gazebo/README.md). 
* `csi_rover_description`: For description please visit [README](src/csi_rover_gazebo/README.md). 
* `csi_rover_gazebo`: For description please visit [README](src/csi_rover_gazebo/README.md). 
* `csi_rover_localizer` - Contains the nodes that support localizing of robot
  from sensor data.

## Policies and Collaboration

Don't commit files in the `build/` or `devel/` directories. These are
automatically produced by catkin and are particular to your specific machine.
Instead, these directories should be left empty and `catkin_make` should be run
when freshly cloning the repository to generate these files.
