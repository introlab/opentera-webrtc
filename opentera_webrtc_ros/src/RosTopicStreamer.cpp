#include <ros/ros.h>
#include <RosTopicStreamer.h>
#include <RosSignalingServerconfiguration.h>

using namespace introlab;
using namespace ros;
using namespace std;

/**
 * @brief construct a topic streamer node
 */
RosTopicStreamer::RosTopicStreamer()
{
    bool needsDenoising;
    bool isScreencast;

    // Load ROS parameters
    loadStreamParams(needsDenoising, isScreencast);

    // Create WebRTC video source and signalling client
    m_videoSource = make_shared<RosVideoSource>(needsDenoising, isScreencast);
    m_signallingClient = make_unique<StreamClient>(
            RosSignalingServerConfiguration::fromRosParam("streamer"),
            WebrtcConfiguration::create(),
            m_videoSource);

    // Subscribe to image topic when signaling client connects
    m_signallingClient->setOnSignallingConnectionOpen([&]{
        ROS_INFO("Signaling connection opened, streaming topic...");
        m_imageSubsriber = m_nh.subscribe(
                "image_raw",
                1,
                &RosVideoSource::imageCallback,
                m_videoSource.get());
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
}

/**
 * @brief Close signaling client connection when this object is destroyed
 */
RosTopicStreamer::~RosTopicStreamer()
{
    ROS_INFO("ROS is shutting down, closing signaling client connection.");
    m_signallingClient->closeSync();
    ROS_INFO("Signaling client disconnected, goodbye.");
}

/**
 * @brief Connect to server and process images forever
 */
void RosTopicStreamer::run()
{
    ROS_INFO("Connecting to signaling server at.");
    m_signallingClient->connect();
    spin();
}

/**
 * @brief Load video stream parameters from ROS parameter server
 *
 * @param denoise whether the images require denoising
 * @param screencast whether the images are a screen capture
 */
void RosTopicStreamer::loadStreamParams(bool &denoise, bool &screencast)
{
    NodeHandle pnh("~stream");

    pnh.param("is_screen_cast", screencast, false);
    pnh.param("needs_denoising", denoise, false);
}

/**
 * @brief runs a ROS topic streamer Node
 *
 * @param argc ROS argument count
 * @param argv ROS argument values
 * @return nothing
 */
int main(int argc, char** argv)
{
    init(argc, argv, "topic_streamer");

    RosTopicStreamer node;
    node.run();
}
