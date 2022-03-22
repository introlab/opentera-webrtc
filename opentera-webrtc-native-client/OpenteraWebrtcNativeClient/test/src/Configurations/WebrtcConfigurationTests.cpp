#include <OpenteraWebrtcNativeClient/Configurations/WebrtcConfiguration.h>
#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(WebrtcConfigurationTests, create_shouldSetTheAttributes)
{
    WebrtcConfiguration testee = WebrtcConfiguration::create();
    EXPECT_EQ(testee.iceServers().size(), 0);
}

TEST(WebrtcConfigurationTests, create_iceServers_shouldSetTheAttributes)
{
    WebrtcConfiguration testee = WebrtcConfiguration::create({IceServer("url1")});
    ASSERT_EQ(testee.iceServers().size(), 1);
    ASSERT_EQ(testee.iceServers()[0].urls().size(), 1);
    EXPECT_EQ(testee.iceServers()[0].urls()[0], "url1");
}

TEST(WebrtcConfigurationTests, operator_webrtcPeerConnectionInterfaceRtcConfiguration_shouldSetTheAttributes)
{
    auto testee = static_cast<webrtc::PeerConnectionInterface::RTCConfiguration>(
        WebrtcConfiguration::create({IceServer("url1")}));
    ASSERT_EQ(testee.servers.size(), 1);
    ASSERT_EQ(testee.servers[0].urls.size(), 1);
    EXPECT_EQ(testee.servers[0].urls[0], "url1");
}
