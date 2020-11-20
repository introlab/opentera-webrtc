#include <RosVideoSource.h>
#include <api/video/i420_buffer.h>

// We use OpenCV for image buffer manipulation
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace introlab;

/**
 * @brief Construct a RosVideoSource
 *
 * @param needsDenoising denoising should be applied to the video stream by the image transport layer
 * @param isScreenCast the transport layer should be configured to stream a screen rather then a camera
 */
RosVideoSource::RosVideoSource(bool needsDenoising, bool isScreenCast):
        VideoSource(needsDenoising, isScreenCast) { }

/**
 * @brief Process a frame received from ROS
 *
 * We grabbed the code from here and added even frame check
 * https://github.com/RobotWebTools/webrtc_ros/blob/develop/webrtc_ros/src/ros_video_capturer.cpp#L48
 *
 * @param msg The ROS image message
 */
void RosVideoSource::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat bgr;
    if (msg->encoding.find("F") != std::string::npos)
    {
        // scale floating point images
        cv::Mat float_image_bridge = cv_bridge::toCvShare(msg, msg->encoding)->image;
        cv::Mat_<float> float_image = float_image_bridge;
        double max_val;
        cv::minMaxIdx(float_image, 0, &max_val);

        if (max_val > 0)
        {
            float_image *= (255 / max_val);
        }
        cv::Mat orig;
        float_image.convertTo(orig, CV_8U);
        cv::cvtColor(orig, bgr, CV_GRAY2BGR);
    }
    else
    {
        bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    }

    int64_t camera_time_us = msg->header.stamp.toNSec() / 1000;
    sendFrame(bgr, camera_time_us);
}
