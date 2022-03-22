#include <OpenteraWebrtcNativeClient/Configurations/SignalingServerConfiguration.h>
#include <OpenteraWebrtcNativeClient/Utils/Client.h>

#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(SignalingServerConfigurationTests, create_urlClientNameRoom_shouldSetTheAttributes)
{
    SignalingServerConfiguration testee = SignalingServerConfiguration::create("url", "name", "room");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_TRUE(*testee.clientData() == *sio::null_message::create());
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "");
}

TEST(SignalingServerConfigurationTests, create_urlClientNameClientDataRoom__shouldSetTheAttributes)
{
    SignalingServerConfiguration testee =
        SignalingServerConfiguration::create("url", "name", sio::int_message::create(10), "room");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_TRUE(*testee.clientData() == *sio::int_message::create(10));
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "");
}

TEST(SignalingServerConfigurationTests, create_urlClientNameRoomPassword_shouldSetTheAttributes)
{
    SignalingServerConfiguration testee = SignalingServerConfiguration::create("url", "name", "room", "password");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_TRUE(*testee.clientData() == *sio::null_message::create());
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "password");
}

TEST(SignalingServerConfigurationTests, create_urlClientNameClientDataRoomPassword_shouldSetTheAttributes)
{
    SignalingServerConfiguration testee =
        SignalingServerConfiguration::create("url", "name", sio::int_message::create(10), "room", "password");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_TRUE(*testee.clientData() == *sio::int_message::create(10));
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "password");
}
