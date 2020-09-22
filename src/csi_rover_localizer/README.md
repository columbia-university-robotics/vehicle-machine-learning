# Rover Localization

## Main Topics

* `/scout_1/ekf_odom` - Publishes Odometry messages with the best location estimate known
* `/scout_1/imu_rect` - Converts NED to ENU (for REP-103 compliance)

## EKF Localization

**To add...**


## Visual Odometry

**STATUS: INTEGRATED ✔️**

### Current status

The visual odometry system uses the rtabmap built-in **RGB-D** odometry feature. Please note that the 
visual odom node **does not** publish the tf between `odom --> base_footprint` (see launch file parameter).
The visual odom node simply publishes a odom message. 
The main localization/odometry node, which is the ekf_odom node, publishes the
tf between odom and base_footprint. 

Note: Because the rover's body and camera link
are not REP-103 compliant, the visual odometry doesn't work currently. Waiting for NASA's patch...

**Published Topic**: `/rtabmap/visual_odom`

**Subscribed Topics**: 

- `/stereo/points2`
- `/stereo/depth_image`
- `/scout_1/camera/left/image_raw_rect`
- `/scout_1/camera/left/camera_info_rect`

The mapping features of rtabmap subscribes to the main odometry topic: `/scout_1/ekf_odom`. 

####  What doesn't work

The rtabmap built-in stereo_odom package doesn't work. After tuning the parameters, the algorithm still had a lot of 
trouble picking up visual features. 

fovis doesn't work. It experienced similar issues as stabmap stereo odometry. There weren't a lot of parameters to tune. 
Not enough visual features were identified. 

## Encoder Based Odometry

**STATUS: INTEGRATED ✔️**

The present strategy makes no assumptions about the angles the wheels
can take (i.e. no Ackermann or differential assumption). To achieve this,
it assumes that the rover starts with all the wheels aligned forward with
the chassis, and then uses the principle of superposition to keep a record
of the angles of the wheels relative to the chassis and the relative
difference in their movement.

