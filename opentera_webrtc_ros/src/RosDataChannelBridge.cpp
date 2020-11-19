#include <ros/ros.h>
#include <RosDataChannelBridge.h>
#include <RosSignalingServerconfiguration.h>

using namespace introlab;
using namespace ros;
using namespace std_msgs;
using namespace std;

/**
 * @brief Construct a data channel bridge not and never returns
 *
 * This function includes a ros::spin() call to keep it from returning.
 */
RosDataChannelBridge::RosDataChannelBridge()
{
    NodeHandle nh;

    // Create signaling client
    m_signallingClient = make_unique<DataChannelClient>(
            RosSignalingServerConfiguration::fromRosParam("data_bridge"),
            WebrtcConfiguration::create(),
            DataChannelConfiguration::create());

    // Advertise topics
    m_dataPublisher = nh.advertise<String>("webrtc_data", 10);
    m_dataSubscriber = nh.subscribe("ros_data", 10, &RosDataChannelBridge::onRosData, this);

    // Setup data channel callback
    m_signallingClient->setOnDataChannelMessageString([&](const Client& client, const string& data) {
        String msg;
        msg.data = data;
        m_dataPublisher.publish(msg);
    });

    // Shutdown ROS when signaling client disconnect
    m_signallingClient->setOnSignallingConnectionClosed([]{
        ROS_WARN("Signaling connection closed, shutting down...");
        requestShutdown();
    });

    // Shutdown ROS on signaling client error
    m_signallingClient->setOnSignallingConnectionError([](auto msg){
        ROS_ERROR("Signaling connection error %s, shutting down...", msg.c_str());
        requestShutdown();
    });

    // Connect to the server
    m_signallingClient->connect();
    spin();
}

/**
 * @brief Close signaling client connection when this object is destroyed
 */
RosDataChannelBridge::~RosDataChannelBridge()
{
    ROS_INFO("ROS is shutting down, closing signaling client connection.");
    m_signallingClient->closeSync();
    ROS_INFO("Signaling client disconnected, goodbye.");
}

/**
 * @brief Send the data received trough ROS to the WebRTC data channel
 *
 * Is used as a callback for the ROS subscriber.
 * Data is sent to all connected peers if the data channel is connected
 *
 * @param msg the received ROS message
 */
void RosDataChannelBridge::onRosData(const StringConstPtr& msg)
{
    m_signallingClient->sendToAll(msg->data);
}

/**
 * @brief runs a ROS data channel bridge
 *
 * @param argc ROS argument count
 * @param argv ROS argument values
 * @return nothing
 */
int main(int argc, char** argv)
{
    init(argc, argv, "data_channel_bridge");
    RosDataChannelBridge();
}
