#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_ROS_TOPIC_STREAMER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_ROS_TOPIC_STREAMER_H

#include <ros/node_handle.h>
#include <RosVideoSource.h>
#include <OpenteraWebrtcNativeClient/StreamClient.h>

namespace introlab {

    /**
     * @brief A ros node that streams a topic to a webrtc call
     *
     * View README.md for more details
     */
    class RosTopicStreamer
    {
        ros::NodeHandle m_nh;
        std::shared_ptr<RosVideoSource> m_videoSource;
        std::unique_ptr<StreamClient> m_signallingClient;
        ros::Subscriber m_imageSubsriber;

        static void loadStreamParams(bool &denoise, bool &screencast);

    public:
        RosTopicStreamer();
        virtual ~RosTopicStreamer();

        void run();
    };

}

#endif
