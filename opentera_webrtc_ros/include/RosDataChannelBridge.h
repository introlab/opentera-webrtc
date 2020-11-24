#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_ROS_DATA_CHANNEL_BRIDGE_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_ROS_DATA_CHANNEL_BRIDGE_H

#include <OpenteraWebrtcNativeClient/DataChannelClient.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <std_msgs/String.h>

namespace introlab {

    /**
     * @brief Implement a ROS node to bridge WebRTC data channel to ROS topics
     */
    class RosDataChannelBridge
    {
        std::unique_ptr<DataChannelClient> m_signallingClient;

        ros::Subscriber m_dataSubscriber;
        ros::Publisher m_dataPublisher;

        void onRosData(const std_msgs::StringConstPtr & msg);

    public:
        RosDataChannelBridge();
        virtual ~RosDataChannelBridge();

        void run();
    };
}

#endif
