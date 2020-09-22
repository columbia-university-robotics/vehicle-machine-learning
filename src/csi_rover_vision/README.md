# Vision Package

# Summary

This package contains all necessary nodes related to computer vision. Currently it's used to launch `stereo_image_proc`

The `stereo_image_proc` package generates a disparity map, depth map, and point cloud from the images from the
stereo cameras on the rover. 

Note: the `camera_link` on the rover is oriented incorrectly, waiting for NASA's patch. 

# Nodes

`camera_info_process`

This package corrects the bugs in the camera_info topics. Please note, we are waiting for NASA to patch this bug. 

# Contacts

Maintained by Noah, Neil, and Sam. 