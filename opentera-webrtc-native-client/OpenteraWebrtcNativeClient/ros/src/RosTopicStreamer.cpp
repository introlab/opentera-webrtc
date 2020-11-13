#include <ros/ros.h>
#include <RosTopicStreamer.h>

using namespace introlab;
using namespace ros;
using namespace std;

class RosSignalingServerConfiguration : public SignallingServerConfiguration
{
public:
    static SignallingServerConfiguration fromParams(NodeHandle &nh)
    {
        string server_url;
        nh.param<string>("server_url", server_url, "http://localhost:8080");

        string client_name;
        nh.param<string>("client_name", client_name, "streamer");

        string room;
        nh.param<string>("room_name", room, "chat");

        string password;
        nh.param<string>("room_password", password, "abc");

        return SignallingServerConfiguration::create(server_url, client_name, room, password);
    }
};

RosTopicStreamer::RosTopicStreamer()
{
    NodeHandle nh;

    string image_topic;
    nh.param<string>("image_topic", image_topic, "camera/image_raw");

    m_videoSource = new RosVideoSource();
    m_signallingClient = make_unique<VideoStreamClient>(
            RosSignalingServerConfiguration::fromParams(nh),
            WebrtcConfiguration::create(),
            m_videoSource);

    m_signallingClient->setOnSignallingConnectionOpen([&]{
        ROS_INFO("Signaling connection opened, streaming topic...");
        m_imageSubsriber = nh.subscribe(
                image_topic,
                1,
                &RosVideoSource::imageCallback,
                m_videoSource.get());
    });

    m_signallingClient->setOnSignallingConnectionClosed([]{
        ROS_INFO("Signaling connection closed, shutting down...");
        requestShutdown();
    });

    m_signallingClient->setOnSignallingConnectionError([](auto msg){
        ROS_ERROR("Signaling connection error %s, shutting down...", msg.c_str());
        requestShutdown();
    });

    ROS_INFO("Connecting to signaling server at.");
    m_signallingClient->connect();

    spin();
}

RosTopicStreamer::~RosTopicStreamer()
{
    ROS_INFO("ROS is shutting down, closing signaling client connection.");
    m_signallingClient->closeSync();
    ROS_INFO("Signaling client disconnected, goodbye.");
}

int main(int argc, char** argv)
{
    init(argc, argv, "topic_streamer");
    RosTopicStreamer();
}
