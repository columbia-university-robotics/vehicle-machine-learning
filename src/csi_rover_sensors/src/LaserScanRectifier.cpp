#include "csi_rover_sensors/LaserScanRectifier.hpp"

namespace rover_sensors {
    LaserScanRectifier::LaserScanRectifier(ros::NodeHandle& nh, int windowSize) : nh_(nh) {
        // Subscribe to raw LaserScan topic
        if (!nh_.getParam("laserscan/subscriber/topic", laserScanSubTopic_)) {
            ROS_ERROR("LaserScanRectifier: Could not read parameters (laserscan/subscriber/topic).");

            ros::requestShutdown();
        }

        laserScanSub_ = nh_.subscribe(laserScanSubTopic_, FREQUENCY, &LaserScanRectifier::laserScanRectifierCallback, this);

        // Create publisher for rectified LaserScan data
        if (!nh_.getParam("laserscan/publisher/topic", laserScanPubTopic_)) {
            ROS_ERROR("LaserScanRectifier: Could not read parameters (laserscan/publisher/topic).");

            ros::requestShutdown();
        }

        laserScanPub_ = nh_.advertise<sensor_msgs::LaserScan>(laserScanPubTopic_, FREQUENCY);

        windowSize_ = windowSize;

        ROS_INFO("LaserScanRectifier: Successfully launched node.");
    }

    void LaserScanRectifier::laserScanRectifierCallback(const sensor_msgs::LaserScanConstPtr& msg) {
        // Save the most recent LaserScan data
        windowData_.push_back(msg);
        
        if (windowData_.size() > windowSize_) {
            windowData_.erase(windowData_.begin());
        }

        // Rectified LaserScan msg
        sensor_msgs::LaserScan msgRect;
        
        // Calculate average values from window data
        float rangesAvg[LASERSCAN_SIZE] = {0};

        for (sensor_msgs::LaserScanConstPtr msgRaw : windowData_) {
            for (int i = 0; i < LASERSCAN_SIZE; ++i) {
                rangesAvg[i] += msgRaw->ranges[i];
            }
        }

        for (int i = 0; i < LASERSCAN_SIZE; ++i) {
            rangesAvg[i] /= windowData_.size();
        } 

        // Build rectified LaserScan msg
        msgRect.header = msg->header;

        msgRect.angle_min = msg->angle_min;
        msgRect.angle_max = msg->angle_max;

        msgRect.angle_increment = msg->angle_increment;
        msgRect.time_increment = msg->time_increment;

        msgRect.scan_time = msg->scan_time;

        msgRect.range_min = msg->range_min;
        msgRect.range_max = msg->range_max;

        msgRect.ranges = std::vector<float>(std::begin(rangesAvg), std::end(rangesAvg));
        msgRect.intensities = msg->intensities;
        
        // Publish rectified LaserScan msg
        laserScanPub_.publish(msgRect);
    }
}