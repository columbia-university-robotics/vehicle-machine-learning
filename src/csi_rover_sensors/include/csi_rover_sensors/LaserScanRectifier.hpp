#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

//Hz
#define FREQUENCY 10
#define LASERSCAN_SIZE 100
#define DEFAULT_LASERSCAN_WINDOW_SIZE 10

namespace rover_sensors {

    class LaserScanRectifier {
        public:
            /**
            * Constructor
            * @param nh the ROS node handle.
            * @param windowSize the size of the mean filter window (default: DEFAULT_LASERSCAN_WINDOW_SIZE)
            */
            LaserScanRectifier(ros::NodeHandle& nh, int windowSize = DEFAULT_LASERSCAN_WINDOW_SIZE);
        
        private:
            /**
            * ROS topic callback method.
            * @param msg the recieved LaserScan message.
            */
            void laserScanRectifierCallback(const sensor_msgs::LaserScanConstPtr& msg);

            // ROS node handle.
            ros::NodeHandle& nh_;

            // ROS topic subscriber. (raw LaserScan)
            ros::Subscriber laserScanSub_;

            // ROS topic name to subscribe to. (raw LaserScan)
            std::string laserScanSubTopic_;

            // ROS topic publisher. (rectified LaserScan)
            ros::Publisher laserScanPub_;

            // ROS topic name to publish to. (rectified LaserScan)
            std::string laserScanPubTopic_;

            // Size of the mean filter window for LaserScan data.
            int windowSize_;

            // Window data for LaserScan data.
            std::vector<sensor_msgs::LaserScanConstPtr> windowData_;
    };

}