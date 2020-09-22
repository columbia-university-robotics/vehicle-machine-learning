# CSI Rover Launcher Package

# Summary

This package is a wrapper on top of other packages inside this project repository. You can think of this package as a 
"phony" "main" package. 
This package will launch all necessary nodes developed by CSI for the SRC challenge (Phase 1). This 
package should only contain launch files and their helper shell scripts.  

The launch file currently depends on those following packages: 

- `csi_rover_controls`
- `csi_rover_vision`
- `csi_rove_localizer`
- `stereo_image_proc`
- `rtabmap_ros`

# Launch Files

There are two launch files, `launch_csi_rover.launch` and `launch_csi_rover_rviz.launch`. The first launch file starts
all nodes. The second launch file depends on the first launch file. 
The second launch file is for visualization, and it simply starts rviz and rqt dashboard. 
Note the there is a parameter inside the localizer launch file for launching the rtabmap visualizer, 
not a part of this package.  

# Contact
Currently maintained by Neil. For questions, issues, feature request, please email Neil at 
[neil.nie@columbia.edu](mailto:neil.nie@columbia.edu)

