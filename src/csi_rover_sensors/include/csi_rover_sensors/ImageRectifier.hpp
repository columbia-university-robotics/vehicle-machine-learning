#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <numeric>

//Hz
#define FREQUENCY 10 
// Minimum value for a white pixel in a grayscale image.
#define MIN_WHITE_PIX_VAL 235
// Minimum number of white pixels to classify as spot noise.
#define WHITE_PIX_NOISE_COUNT 100
// The minimum percentage of desired saturation values 
#define SATURATION_PERC 0.05
#define DEFAULT_IMAGE_WINDOW_SIZE 15

namespace rover_sensors {

    class ImageRectifier {
        public:
            /**
            * Constructor
            * @param nh the ROS node handle.
            * @param windowSize the size of the mean filter window (default: DEFAULT_IMAGE_WINDOW_SIZE)
            */
            ImageRectifier(ros::NodeHandle& nh, int windowSize = DEFAULT_IMAGE_WINDOW_SIZE);
        
        private:
            /**
            * ROS topic callback method.
            * @param msg the recieved left Image message.
            */
            void leftImageCallback(const sensor_msgs::ImageConstPtr& msg);

            /**
            * ROS topic callback method.
            * @param msg the recieved right Image message.
            */
            void rightImageCallback(const sensor_msgs::ImageConstPtr& msg);

            /**
             * Method to determine if an image contains dot noise.
             * @param image the raw image
             */
            bool imageContainsDotNoise(const cv::Mat image);

            /**
             * Method to determine if an image contains spot noise.
             * @param image the raw image
             */
            bool imageContainsSpotNoise(const cv::Mat image);

            /**
             * Method to determine if an image contains noise (either dot or spot).
             * @param image the raw image
             */
            bool imageContainsNoise(const cv::Mat image);

            // ROS node handle.
            ros::NodeHandle& nh_;

            // ROS topic subscriber. (raw left Image)
            ros::Subscriber leftImageSub_;

            // ROS topic subscriber. (raw right Image)
            ros::Subscriber rightImageSub_;

            // ROS topic publisher. (rectified left Image)
            ros::Publisher leftImagePub_;

            // ROS topic publisher. (rectified right Image)
            ros::Publisher rightImagePub_;

            // ROS topic name to subscribe to. (raw left Image)
            std::string leftImageSubTopic_;

            // ROS topic name to subscribe to. (raw right Image)
            std::string rightImageSubTopic_;

            // ROS topic name to publish to. (rectified left Image)
            std::string leftImagePubTopic_;

            // ROS topic name to publish to. (rectified right Image)
            std::string rightImagePubTopic_;
            
            // Size of the window of saturation percentages for left and right Images
            int windowSize_;
            
            // Window data for saturation percentages for left and right Images
            std::vector<float> windowData_;
    };

}