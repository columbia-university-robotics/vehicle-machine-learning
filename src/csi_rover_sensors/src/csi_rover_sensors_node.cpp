#include <ros/ros.h>
#include "csi_rover_sensors/IMURectifier.hpp"
#include "csi_rover_sensors/LaserScanRectifier.hpp"
#include "csi_rover_sensors/ImageRectifier.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rover_sensors_node");
    
    ros::NodeHandle nh("~");

    rover_sensors::IMURectifier imuRect(nh);
    rover_sensors::LaserScanRectifier laserScanRect(nh);
    //rover_sensors::ImageRectifier imageRect(nh);

    ros::spin();

    return 0;
}