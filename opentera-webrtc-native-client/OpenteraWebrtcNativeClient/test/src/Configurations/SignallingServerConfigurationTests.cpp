#include <OpenteraWebrtcNativeClient/Configurations/SignallingServerConfiguration.h>
#include <OpenteraWebrtcNativeClient/Utils/Client.h>

#include <gtest/gtest.h>

using namespace introlab;
using namespace std;

TEST(SignallingServerConfigurationTests, create_urlClientNameRoom_shouldSetTheAttributes)
{
    SignallingServerConfiguration testee = SignallingServerConfiguration::create("url", "name", "room");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_TRUE(*testee.clientData() == *sio::null_message::create());
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "");
}

TEST(SignallingServerConfigurationTests, create_urlClientNameClientDataRoom__shouldSetTheAttributes)
{
    SignallingServerConfiguration testee = SignallingServerConfiguration::create("url", "name",
            sio::int_message::create(10), "room");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_TRUE(*testee.clientData() == *sio::int_message::create(10));
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "");
}

TEST(SignallingServerConfigurationTests, create_urlClientNameRoomPassword_shouldSetTheAttributes)
{
    SignallingServerConfiguration testee = SignallingServerConfiguration::create("url", "name", "room", "password");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_TRUE(*testee.clientData() == *sio::null_message::create());
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "password");
}

TEST(SignallingServerConfigurationTests, create_urlClientNameClientDataRoomPassword_shouldSetTheAttributes)
{
    SignallingServerConfiguration testee = SignallingServerConfiguration::create("url", "name",
        sio::int_message::create(10), "room", "password");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_TRUE(*testee.clientData() == *sio::int_message::create(10));
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "password");
}
