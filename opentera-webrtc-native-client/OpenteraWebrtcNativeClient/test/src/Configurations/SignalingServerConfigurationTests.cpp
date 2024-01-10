#include <OpenteraWebrtcNativeClient/Configurations/SignalingServerConfiguration.h>
#include <OpenteraWebrtcNativeClient/Utils/Client.h>

#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(SignalingServerConfigurationTests, create_urlClientNameRoom_shouldSetTheAttributes)
{
    auto testee = SignalingServerConfiguration::create("url", "name", "room");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_EQ(testee.clientData(), nlohmann::json{});
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "");
}

TEST(SignalingServerConfigurationTests, createWithData_urlClientNameClientDataRoom__shouldSetTheAttributes)
{
    auto testee = SignalingServerConfiguration::createWithData("url", "name", 10, "room");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_EQ(testee.clientData(), 10);
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "");
}

TEST(SignalingServerConfigurationTests, create_urlClientNameRoomPassword_shouldSetTheAttributes)
{
    auto testee = SignalingServerConfiguration::create("url", "name", "room", "password");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_EQ(testee.clientData(), nlohmann::json{});
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "password");
}

TEST(SignalingServerConfigurationTests, createWithData_urlClientNameClientDataRoomPassword_shouldSetTheAttributes)
{
    auto testee = SignalingServerConfiguration::createWithData("url", "name", 10, "room", "password");

    EXPECT_EQ(testee.url(), "url");
    EXPECT_EQ(testee.clientName(), "name");
    EXPECT_EQ(testee.clientData(), 10);
    EXPECT_EQ(testee.room(), "room");
    EXPECT_EQ(testee.password(), "password");
}
