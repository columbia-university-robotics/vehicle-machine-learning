#include "csi_rover_sensors/ImageRectifier.hpp"

namespace rover_sensors {
    ImageRectifier::ImageRectifier(ros::NodeHandle& nh, int windowSize) : nh_(nh) {
        // Subscribe to raw left Image topic
        if (!nh_.getParam("camera/left/subscriber/topic", leftImageSubTopic_)) {
            ROS_ERROR("ImageRectifier: Could not read parameters (camera/left/subscriber/topic).");

            ros::requestShutdown();
        }

        leftImageSub_ = nh_.subscribe(leftImageSubTopic_, 10, &ImageRectifier::leftImageCallback, this);

        // Create publisher for rectified left Image data
        if (!nh_.getParam("camera/left/publisher/topic", leftImagePubTopic_)) {
            ROS_ERROR("ImageRectifier: Could not read parameters (camera/left/publisher/topic).");

            ros::requestShutdown();
        }

        leftImagePub_ = nh_.advertise<sensor_msgs::Image>(leftImagePubTopic_, 10);

        // Subscribe to raw right Image topic
        if (!nh_.getParam("camera/right/subscriber/topic", rightImageSubTopic_)) {
            ROS_ERROR("ImageRectifier: Could not read parameters (camera/right/subscriber/topic).");

            ros::requestShutdown();
        }

        rightImageSub_ = nh_.subscribe(rightImageSubTopic_, 10, &ImageRectifier::rightImageCallback, this);

        // Create publisher for rectified right Image data
        if (!nh_.getParam("camera/right/publisher/topic", rightImagePubTopic_)) {
            ROS_ERROR("ImageRectifier: Could not read parameters (camera/right/publisher/topic).");

            ros::requestShutdown();
        }

        rightImagePub_ = nh_.advertise<sensor_msgs::Image>(rightImagePubTopic_, 10);

        windowSize_ = windowSize;

        ROS_INFO("ImageRectifier: Successfully launched node.");
    }

    void ImageRectifier::leftImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // Convert sensor_msg::Image to cv_bridge::CvImage
        cv_bridge::CvImagePtr cvImagePtr;

        try {
            cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }

        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());

            return;
        }

        // Extract cv::Mat from cv_bridge::CvImage
        cv::Mat image = cvImagePtr->image;

        // Publish Image msg if it contains no noise
        if (!imageContainsNoise(image)) {
            leftImagePub_.publish(msg);
        }
    }

    void ImageRectifier::rightImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // Convert sensor_msg::Image to cv_bridge::CvImage
        cv_bridge::CvImagePtr cvImagePtr;

        try {
            cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }

        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());

            return;
        }

        // Extract cv::Mat from cv_bridge::CvImage
        cv::Mat image = cvImagePtr->image;

        // Publish Image msg if it contains no noise
        if (!imageContainsNoise(image)) {
            rightImagePub_.publish(msg);
        }
    }

    bool ImageRectifier::imageContainsDotNoise(const cv::Mat image) {
        // Convert RGB image to HSV
        cv::Mat imageHsv;
        cv::cvtColor(image, imageHsv, cv::COLOR_BGR2HSV);

        // Calculate a histogram of saturation values for the HSV image
        cv::Mat hist;
        const int histSize = 256;
        const int channels = 1;
        float range[] = {0, 256};
        const float* histRange = {range};
        cv::calcHist(&imageHsv, 1, &channels, cv::Mat(), hist, 1, &histSize, &histRange);

        // Calculate the % of pixels with saturation values in the top (1 - SATURATION_PERC) %  
        float saturationPercent = 0;
        for (int i = int(255 * SATURATION_PERC); i < histSize; ++i) {
            saturationPercent += hist.at<float>(i);
        }
        saturationPercent /= (image.rows * image.cols);
        
        // Save the most recent saturation percent
        windowData_.push_back(saturationPercent);

        if (windowData_.size() > windowSize_) {
            windowData_.erase(windowData_.begin());
        }

        // Calculate mean of saturation percentanges in windowData_
        float sum = std::accumulate(windowData_.begin(), windowData_.end(), 0.0);
        float mean = sum / windowData_.size();

        // If current saturation percent is above the mean saturation percent, the image contains dot noise
        return saturationPercent >= mean;
    }

    bool ImageRectifier::imageContainsSpotNoise(const cv::Mat image) {
        // Convert RGB image to grayscale
        cv::Mat imageGray;
        cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);

        // Find mask of white pixels in a grayscale image
        cv::Mat whiteMask;
        cv::threshold(imageGray, whiteMask, MIN_WHITE_PIX_VAL, 255, cv::THRESH_BINARY);

        // Find all white pixels in mask
        std::vector<cv::Point> whitePixels;
        cv::findNonZero(whiteMask, whitePixels);
        
        // If the number of white pixels exceeds WHITE_PIX_NOISE_THRESHOLD, the image contains spot noise
        return whitePixels.size() >= WHITE_PIX_NOISE_COUNT;
    }

    bool ImageRectifier::imageContainsNoise(const cv::Mat image) {
        // The image contains noise if it has either dot or spot noise
        return imageContainsDotNoise(image) || imageContainsSpotNoise(image);
    }
}