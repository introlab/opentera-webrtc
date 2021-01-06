#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_ROS_TOPIC_STREAMER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_ROS_TOPIC_STREAMER_H

#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <RosVideoSource.h>
#include <OpenteraWebrtcNativeClient/StreamClient.h>
#include <OpenteraWebrtcNativeClient/Sinks/VideoSink.h>

namespace introlab {

    /**
     * @brief A ros node that bridges streams with ROS topics
     *
     * View README.md for more details
     */
    class RosStreamBridge
    {
        ros::NodeHandle m_nh;
        std::shared_ptr<RosVideoSource> m_videoSource;
        std::shared_ptr<VideoSink> m_videoSink;
        std::unique_ptr<StreamClient> m_signallingClient;

        ros::Subscriber m_imageSubsriber;
        ros::Publisher m_imagePublisher;

        static void loadStreamParams(bool &denoise, bool &screencast);
        void onFrameReceived(const cv::Mat& bgrImg, uint64_t timestampUs);

    public:
        RosStreamBridge();
        virtual ~RosStreamBridge();

        void run();
    };

}

#endif
