#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

#include <gtest/gtest.h>

using namespace introlab;
using namespace std;

TEST(IceServerTests, constructor_url_shouldSetTheAttributes)
{
    IceServer testee("url1");

    ASSERT_EQ(testee.urls().size(), 1);
    EXPECT_EQ(testee.urls()[0], "url1");
    EXPECT_EQ(testee.username(), "");
    EXPECT_EQ(testee.credential(), "");
}

TEST(IceServerTests, constructor_urlUsernameCredential_shouldSetTheAttributes)
{
    IceServer testee("url1", "user", "password");

    ASSERT_EQ(testee.urls().size(), 1);
    EXPECT_EQ(testee.urls()[0], "url1");
    EXPECT_EQ(testee.username(), "user");
    EXPECT_EQ(testee.credential(), "password");
}

TEST(IceServerTests, constructor_urls_shouldSetTheAttributes)
{
    IceServer testee(vector<string>({"url1", "url2"}));

    ASSERT_EQ(testee.urls().size(), 2);
    EXPECT_EQ(testee.urls()[0], "url1");
    EXPECT_EQ(testee.urls()[1], "url2");
    EXPECT_EQ(testee.username(), "");
    EXPECT_EQ(testee.credential(), "");
}

TEST(IceServerTests, constructor_urlsUsernameCredential_shouldSetTheAttributes)
{
    IceServer testee(vector<string>({"url1", "url2"}), "user", "password");

    ASSERT_EQ(testee.urls().size(), 2);
    EXPECT_EQ(testee.urls()[0], "url1");
    EXPECT_EQ(testee.urls()[1], "url2");
    EXPECT_EQ(testee.username(), "user");
    EXPECT_EQ(testee.credential(), "password");
}

TEST(IceServerTests, operator_webrtcIceServer_shouldSetTheAttributes)
{
    auto testee = static_cast<webrtc::PeerConnectionInterface::IceServer>(IceServer(vector<string>({"u"}), "s", "p"));

    ASSERT_EQ(testee.urls.size(), 1);
    EXPECT_EQ(testee.urls[0], "u");
    EXPECT_EQ(testee.username, "s");
    EXPECT_EQ(testee.password, "p");
}
