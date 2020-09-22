#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#define FREQUENCY 10 //Hz

#define DEFAULT_IMU_WINDOW_SIZE 50

namespace rover_sensors {

    class IMURectifier {
        public:
            /**
            * Constructor
            * @param nh the ROS node handle.
            * @param windowSize the size of the mean filter window (default: DEFAULT_IMU_WINDOW_SIZE)
            */
            IMURectifier(ros::NodeHandle& nh, int windowSize = DEFAULT_IMU_WINDOW_SIZE);
        
        private:
            /**
            * ROS topic callback method.
            * @param msg the recieved IMU message.
            */
            void imuRectifierCallback(const sensor_msgs::ImuConstPtr& msg);

            // ROS node handle.
            ros::NodeHandle& nh_;

            // ROS topic subscriber. (raw IMU)
            ros::Subscriber imuSub_;

            // ROS topic name to subscribe to. (raw IMU)
            std::string imuSubTopic_;

            // ROS topic publisher. (rectified IMU)
            ros::Publisher imuPub_;

            // ROS topic name to publish to. (rectified IMU)
            std::string imuPubTopic_;

            // Size of the mean filter window for IMU data.
            int windowSize_;

            // Window data for IMU data.
            std::vector<sensor_msgs::ImuConstPtr> windowData_;
    };

}