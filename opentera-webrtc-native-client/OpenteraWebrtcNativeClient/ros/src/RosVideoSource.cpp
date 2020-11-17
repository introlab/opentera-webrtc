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
        m_needsDenoising(needsDenoising), m_isScreenCast(isScreenCast) { }

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
    cv::Rect roi;
    int out_width, out_height;

    // AdaptFrame return true if the transport layer needs a frame
    // Desired resolution is set in out_width and out_height
    if (AdaptFrame(bgr.cols, bgr.rows, camera_time_us, &out_width, &out_height,
                   &roi.width, &roi.height, &roi.x, &roi.y))
    {
        // I420 only support even resolution so we must make out resolution even!
        out_width = (out_width / 2) * 2;
        out_height = (out_height / 2) * 2;

        cv::Mat yuv;
        if (out_width == roi.width && out_height == roi.height)
        {
            cv::cvtColor(bgr(roi), yuv, CV_BGR2YUV_I420);
        }
        else
        {
            cv::Mat m;
            cv::resize(bgr(roi), m, cv::Size2i(out_width, out_height), 0, 0,
                       out_width < roi.width ? cv::INTER_AREA : cv::INTER_LINEAR);
            cv::cvtColor(m, yuv, CV_BGR2YUV_I420);
        }

        uint8_t *y = yuv.data;
        uint8_t *u = y + (out_width * out_height);
        uint8_t *v = u + (out_width * out_height) / 4;
        webrtc::VideoFrame frame(
                webrtc::I420Buffer::Copy(out_width, out_height, y, out_width,
                                         u, out_width / 2, v, out_width / 2),
                webrtc::kVideoRotation_0,
                camera_time_us
        );

        // Passes the frame to the transport layer
        OnFrame(frame);
    }
}
