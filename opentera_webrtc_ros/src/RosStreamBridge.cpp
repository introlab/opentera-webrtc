#include <ros/ros.h>
#include <RosStreamBridge.h>
#include <RosSignalingServerconfiguration.h>
#include <cv_bridge/cv_bridge.h>

using namespace introlab;
using namespace ros;
using namespace std;

/**
 * @brief construct a topic streamer node
 */
RosStreamBridge::RosStreamBridge()
{
    bool needsDenoising;
    bool isScreencast;

    // Load ROS parameters
    loadStreamParams(needsDenoising, isScreencast);

    // WebRTC video stream interfaces
    m_videoSource = make_shared<RosVideoSource>(needsDenoising, isScreencast);
    m_videoSink = make_shared<VideoSink>([&](const cv::Mat& bgrImg, uint64_t timestampUs){
        onFrameReceived(bgrImg, timestampUs);
    });

    // Signaling client connection
    m_signallingClient = make_unique<StreamClient>(
            RosSignalingServerConfiguration::fromRosParam("streamer"),
            WebrtcConfiguration::create(),
            m_videoSource,
            m_videoSink);

    m_imagePublisher = m_nh.advertise<sensor_msgs::Image>("webrtc_image", 1, false);

    // Subscribe to image topic when signaling client connects
    m_signallingClient->setOnSignallingConnectionOpen([&]{
        ROS_INFO("Signaling connection opened, streaming topic...");
        m_imageSubsriber = m_nh.subscribe(
                "ros_image",
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
 * @brief publish an image using the node image publisher
 *
 * @param bgrImg BGR8 encoded image
 * @param timestampUs image timestamp in microseconds
 */
void RosStreamBridge::onFrameReceived(const cv::Mat& bgrImg, uint64_t timestampUs)
{
    std_msgs::Header imgHeader;
    imgHeader.stamp.fromNSec(1000 * timestampUs);

    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(imgHeader, "bgr8", bgrImg).toImageMsg();
    m_imagePublisher.publish(imgMsg);
}

/**
 * @brief Close signaling client connection when this object is destroyed
 */
RosStreamBridge::~RosStreamBridge()
{
    ROS_INFO("ROS is shutting down, closing signaling client connection.");
    m_signallingClient->closeSync();
    ROS_INFO("Signaling client disconnected, goodbye.");
}

/**
 * @brief Connect to server and process images forever
 */
void RosStreamBridge::run()
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
void RosStreamBridge::loadStreamParams(bool &denoise, bool &screencast)
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
    init(argc, argv, "stream_bridge");

    RosStreamBridge node;
    node.run();
}
