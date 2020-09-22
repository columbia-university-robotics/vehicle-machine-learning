#include "csi_rover_sensors/IMURectifier.hpp"

namespace rover_sensors {
    IMURectifier::IMURectifier(ros::NodeHandle& nh, int windowSize) : nh_(nh) {
        // Subscribe to raw IMU topic
        if (!nh_.getParam("imu/subscriber/topic", imuSubTopic_)) {
            ROS_ERROR("IMURectifier: Could not read parameters (imu/subscriber/topic).");
            
            ros::requestShutdown();
        }

        imuSub_ = nh_.subscribe(imuSubTopic_, FREQUENCY, &IMURectifier::imuRectifierCallback, this);

        // Create publisher for rectified IMU data
        if (!nh_.getParam("imu/publisher/topic", imuPubTopic_)) {
            ROS_ERROR("IMURectifier: Could not read parameters (imu/publisher/topic).");
            
            ros::requestShutdown();
        }

        imuPub_ = nh_.advertise<sensor_msgs::Imu>(imuPubTopic_, FREQUENCY);

        windowSize_ = windowSize;

        ROS_INFO("IMURectifier: Successfully launched node.");
    }

    void IMURectifier::imuRectifierCallback(const sensor_msgs::ImuConstPtr& msg) {
        // Save the most recent IMU data
        windowData_.push_back(msg);
        
        if (windowData_.size() > windowSize_) {
            windowData_.erase(windowData_.begin());
        }

        // Rectified IMU msg
        sensor_msgs::Imu msgRect;

        // Calculate average values from window data
        float orienXAvg = 0;
        float orienYAvg = 0;
        float orienZAvg = 0;
        float orienWAvg = 0;

        float angVelXAvg = 0;
        float angVelYAvg = 0;
        float angVelZAvg = 0;

        float linAccXAvg = 0;
        float linAccYAvg = 0;
        float linAccZAvg = 0;

        for (sensor_msgs::ImuConstPtr msgRaw : windowData_) {
            orienXAvg += msgRaw->orientation.x;
            orienYAvg += msgRaw->orientation.y;
            orienZAvg += msgRaw->orientation.z;
            orienWAvg += msgRaw->orientation.w;

            angVelXAvg += msgRaw->angular_velocity.x;
            angVelYAvg += msgRaw->angular_velocity.y;
            angVelZAvg += msgRaw->angular_velocity.z;

            linAccXAvg += msgRaw->linear_acceleration.x;
            linAccYAvg += msgRaw->linear_acceleration.y;
            linAccZAvg += msgRaw->linear_acceleration.z;
        }

        orienXAvg = orienXAvg / windowData_.size();
        orienYAvg = orienYAvg / windowData_.size();
        orienZAvg = orienZAvg / windowData_.size();
        orienWAvg = orienWAvg / windowData_.size();

        angVelXAvg = angVelXAvg / windowData_.size();
        angVelYAvg = angVelYAvg / windowData_.size();
        angVelZAvg = angVelZAvg / windowData_.size();

        linAccXAvg = linAccXAvg / windowData_.size();
        linAccYAvg = linAccYAvg / windowData_.size();
        linAccZAvg = linAccZAvg / windowData_.size();

        // Build rectified IMU msg
        msgRect.header = msg->header;

        msgRect.orientation.x = orienXAvg;
        msgRect.orientation.y = orienYAvg;
        msgRect.orientation.z = orienZAvg;
        msgRect.orientation.w = orienWAvg;

        msgRect.orientation_covariance = msg->orientation_covariance;

        msgRect.angular_velocity.x = angVelXAvg;
        msgRect.angular_velocity.y = angVelYAvg;
        msgRect.angular_velocity.z = angVelZAvg;

        msgRect.angular_velocity_covariance = msg->angular_velocity_covariance;

        msgRect.linear_acceleration.x = linAccXAvg;
        msgRect.linear_acceleration.y = linAccYAvg;
        msgRect.linear_acceleration.z = linAccZAvg;

        msgRect.linear_acceleration_covariance = msg->linear_acceleration_covariance;

        // Publish rectified IMU msg
        imuPub_.publish(msgRect);
    }
}