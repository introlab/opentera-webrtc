#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

#include <gtest/gtest.h>

#include <subprocess.hpp>

#include <boost/filesystem.hpp>

using namespace opentera;
using namespace std;
namespace fs = boost::filesystem;

class IceServerTestsWithSignalingServer : public ::testing::Test
{
    static unique_ptr<subprocess::Popen> m_signalingServerProcess;

protected:
    static void SetUpTestSuite()
    {
        fs::path testFilePath(__FILE__);
        fs::path pythonFilePath = testFilePath.parent_path().parent_path().parent_path().parent_path().parent_path()
                .parent_path() / "signaling-server" / "signaling_server.py";
        m_signalingServerProcess = make_unique<subprocess::Popen>("python3 " + pythonFilePath.string() +
                " --port 8080 --password abc --ice_servers resources/iceServers.json",
                subprocess::input(subprocess::PIPE));
        this_thread::sleep_for(500ms);
    }

    static void TearDownTestSuite()
    {
        if (m_signalingServerProcess)
        {
            m_signalingServerProcess->kill(9);
            m_signalingServerProcess->wait();
        }
    }
};
unique_ptr<subprocess::Popen> IceServerTestsWithSignalingServer::m_signalingServerProcess = nullptr;

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

TEST_F(IceServerTestsWithSignalingServer, fetchFromServer_invalidUrl_shouldReturnTrueAndNotSetIceServers)
{
    vector<IceServer> iceServers;
    EXPECT_FALSE(IceServer::fetchFromServer("http://localhost:8080/ice", "", iceServers));
    ASSERT_EQ(iceServers.size(), 0);
}

TEST_F(IceServerTestsWithSignalingServer, fetchFromServer_wrongPassword_shouldReturnTrueAndNotSetIceServers)
{
    vector<IceServer> iceServers;
    EXPECT_TRUE(IceServer::fetchFromServer("http://localhost:8080/iceservers", "", iceServers));
    ASSERT_EQ(iceServers.size(), 0);
}

TEST_F(IceServerTestsWithSignalingServer, fetchFromServer_rightPassword_shouldReturnTrueAndSetIceServers)
{
    vector<IceServer> iceServers;
    EXPECT_TRUE(IceServer::fetchFromServer("http://localhost:8080/iceservers", "abc", iceServers));
    ASSERT_EQ(iceServers.size(), 1);
    ASSERT_EQ(iceServers[0].urls().size(), 1);
    EXPECT_EQ(iceServers[0].urls()[0], "stun:stun.l.google.com:19302");
    EXPECT_EQ(iceServers[0].username(), "");
    EXPECT_EQ(iceServers[0].credential(), "");
}

TEST(IceServerTests, fromJson_invalid_shouldReturnFalse)
{
    const string invalidJson0 = "{}";
    const string invalidJson1 = "[0]";
    const string invalidJson2 = "[{}]";
    const string invalidJson3 = "[{\"urls\":0}]";
    const string invalidJson4 = "[{\"urls\": [0]}]";
    const string invalidJson5 = "[{\"urls\": [\"\"], \"username\": 0, \"credential\": 0}]";
    const string invalidJson6 = "[{\"urls\": [\"\"], \"username\": \"\", \"credential\": 0}]";
    const string invalidJson7 = "[{\"urls\": [\"\"], \"username\": 0, \"credential\": \"\"}]";
    const string invalidJson8 = "[{\"urls\": [\"\"], \"username\": \"\"}]";
    const string invalidJson9 = "[{\"urls\": [\"\"], \"credential\": \"\"}]";
    vector<IceServer> iceServers;

    EXPECT_FALSE(IceServer::fromJson(invalidJson0, iceServers));
    EXPECT_FALSE(IceServer::fromJson(invalidJson1, iceServers));
    EXPECT_FALSE(IceServer::fromJson(invalidJson2, iceServers));
    EXPECT_FALSE(IceServer::fromJson(invalidJson3, iceServers));
    EXPECT_FALSE(IceServer::fromJson(invalidJson4, iceServers));
    EXPECT_FALSE(IceServer::fromJson(invalidJson5, iceServers));
    EXPECT_FALSE(IceServer::fromJson(invalidJson6, iceServers));
    EXPECT_FALSE(IceServer::fromJson(invalidJson7, iceServers));
    EXPECT_FALSE(IceServer::fromJson(invalidJson8, iceServers));
    EXPECT_FALSE(IceServer::fromJson(invalidJson9, iceServers));
}

TEST(IceServerTests, fromJson_valid_shouldReturnTrueAndSetIceServers)
{
    const string validJson0 = "[]";
    const string validJson1 = "[{\"urls\": \"a\"}]";
    const string validJson2 = "[{\"urls\": [\"a\", \"b\"]}]";
    const string validJson3 = "[{\"urls\": \"a\", \"username\": \"u\", \"credential\": \"c\"}]";
    vector<IceServer> iceServers;

    EXPECT_TRUE(IceServer::fromJson(validJson0, iceServers));
    ASSERT_EQ(iceServers.size(), 0);

    iceServers.clear();
    EXPECT_TRUE(IceServer::fromJson(validJson1, iceServers));
    ASSERT_EQ(iceServers.size(), 1);
    ASSERT_EQ(iceServers[0].urls().size(), 1);
    EXPECT_EQ(iceServers[0].urls()[0], "a");
    EXPECT_EQ(iceServers[0].username(), "");
    EXPECT_EQ(iceServers[0].credential(), "");

    iceServers.clear();
    EXPECT_TRUE(IceServer::fromJson(validJson2, iceServers));
    ASSERT_EQ(iceServers.size(), 1);
    ASSERT_EQ(iceServers[0].urls().size(), 2);
    EXPECT_EQ(iceServers[0].urls()[0], "a");
    EXPECT_EQ(iceServers[0].urls()[1], "b");
    EXPECT_EQ(iceServers[0].username(), "");
    EXPECT_EQ(iceServers[0].credential(), "");

    iceServers.clear();
    EXPECT_TRUE(IceServer::fromJson(validJson3, iceServers));
    ASSERT_EQ(iceServers.size(), 1);
    ASSERT_EQ(iceServers[0].urls().size(), 1);
    EXPECT_EQ(iceServers[0].urls()[0], "a");
    EXPECT_EQ(iceServers[0].username(), "u");
    EXPECT_EQ(iceServers[0].credential(), "c");
}
