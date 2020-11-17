#include <ros/ros.h>
#include <RosTopicStreamer.h>

using namespace introlab;
using namespace ros;
using namespace std;

/**
 * @brief construct a topic streamer node
 */
RosTopicStreamer::RosTopicStreamer()
{
    NodeHandle nh;
    string imageTopic;
    bool needsDenoising;
    bool isScreencast;

    // Load ROS parameters
    loadStreamParams(imageTopic, needsDenoising, isScreencast);
    SignallingServerConfiguration signalingConfig = loadSignalingConfig();

    // Create WebRTC video source and signalling client
    rtc::scoped_refptr<RosVideoSource> videoSource = new RosVideoSource(needsDenoising, isScreencast);
    m_signallingClient = make_unique<VideoStreamClient>(
            signalingConfig,
            WebrtcConfiguration::create(),
            videoSource);

    // Subscribe to image topic when signaling client connects
    m_signallingClient->setOnSignallingConnectionOpen([&]{
        ROS_INFO("Signaling connection opened, streaming topic...");
        m_imageSubsriber = nh.subscribe(
                imageTopic,
                1,
                &RosVideoSource::imageCallback,
                videoSource.get());
    });

    // Shutdown ROS when signaling client disconnect
    m_signallingClient->setOnSignallingConnectionClosed([]{
        ROS_INFO("Signaling connection closed, shutting down...");
        requestShutdown();
    });

    // Shutdown ROS on signaling client error
    m_signallingClient->setOnSignallingConnectionError([](auto msg){
        ROS_ERROR("Signaling connection error %s, shutting down...", msg.c_str());
        requestShutdown();
    });

    // Connect to server and process images forever
    ROS_INFO("Connecting to signaling server at.");
    m_signallingClient->connect();
    spin();
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
 * @brief Load video stream parameters from ROS parameter server
 * 
 * @param topic image topic to subscribe to
 * @param denoise whether the images require denoising
 * @param screencast whether the images are a screen capture
 */
void RosTopicStreamer::loadStreamParams(std::string &topic, bool &denoise, bool &screencast)
{
    NodeHandle pnh("~stream");

    pnh.param<string>("topic", topic, "camera/image_raw");
    pnh.param("is_screen_cast", screencast, false);
    pnh.param("needs_denoising", denoise, false);
}

/**
 * @brief Load signaling server configuration from ROS parameter server
 *
 * @return the signaling server configuration object
 */
SignallingServerConfiguration RosTopicStreamer::loadSignalingConfig()
{
    NodeHandle pnh("~signaling");

    string server_url;
    pnh.param<string>("server_url", server_url, "http://localhost:8080");

    string client_name;
    pnh.param<string>("client_name", client_name, "streamer");

    string room;
    pnh.param<string>("room_name", room, "chat");

    string password;
    pnh.param<string>("room_password", password, "abc");

    return SignallingServerConfiguration::create(server_url, client_name, room, password);
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
    RosTopicStreamer();
}
