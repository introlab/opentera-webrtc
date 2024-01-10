#include <OpenteraWebrtcNativeClient/DataChannelClient.h>

#include <OpenteraWebrtcNativeClientTests/CallbackAwaiter.h>

#include <gtest/gtest.h>

#include <subprocess.hpp>

#include <filesystem>

#include <memory>
#include <thread>

using namespace opentera;
using namespace std;
namespace fs = std::filesystem;

static const WebrtcConfiguration DefaultWebrtcConfiguration =
    WebrtcConfiguration::create({IceServer("stun:stun.l.google.com:19302")});

class DataChannelClientTests : public ::testing::TestWithParam<bool>
{
    static unique_ptr<subprocess::Popen> m_signalingServerProcessTLS;
    static unique_ptr<subprocess::Popen> m_signalingServerProcess;

protected:
    string m_baseUrl;

    static void SetUpTestSuite()
    {
        fs::path testFilePath(__FILE__);
        fs::path pythonFilePath = testFilePath.parent_path().parent_path().parent_path().parent_path().parent_path() /
                                  "signaling-server" / "opentera-signaling-server";

        m_signalingServerProcessTLS = make_unique<subprocess::Popen>(
            "python3 " + pythonFilePath.string() +
                " --port 8081 --password abc"
                " --certificate resources/cert.pem"
                " --key resources/key.pem",
            subprocess::input(subprocess::PIPE));

        m_signalingServerProcess = make_unique<subprocess::Popen>(
            "python3 " + pythonFilePath.string() + " --port 8080 --password abc",
            subprocess::input(subprocess::PIPE));

        this_thread::sleep_for(2s);
    }

    static void TearDownTestSuite()
    {
        if (m_signalingServerProcessTLS)
        {
            m_signalingServerProcessTLS->kill(9);
            m_signalingServerProcessTLS->wait();
        }

        if (m_signalingServerProcess)
        {
            m_signalingServerProcess->kill(9);
            m_signalingServerProcess->wait();
        }
    }

    void SetUp() override
    {
        bool tlsTestEnable = GetParam();

        if (tlsTestEnable)
        {
            m_baseUrl = "wss://localhost:8081/signaling";
        }
        else
        {
            m_baseUrl = "ws://localhost:8080/signaling";
        }
    }
};

unique_ptr<subprocess::Popen> DataChannelClientTests::m_signalingServerProcess = nullptr;
unique_ptr<subprocess::Popen> DataChannelClientTests::m_signalingServerProcessTLS = nullptr;

class DisconnectedDataChannelClientTests : public ::testing::TestWithParam<bool>
{
protected:
    unique_ptr<DataChannelClient> m_client1;

    void SetUp() override
    {
        bool tlsTestEnable = GetParam();

        if (tlsTestEnable)
        {
            m_client1 = make_unique<DataChannelClient>(
                SignalingServerConfiguration::createWithData("wss://localhost:8081/signaling", "c1", "cd1", "chat", ""),
                DefaultWebrtcConfiguration,
                DataChannelConfiguration::create());
        }
        else
        {
            m_client1 = make_unique<DataChannelClient>(
                SignalingServerConfiguration::createWithData("ws://localhost:8080/signaling", "c1", "cd1", "chat", ""),
                DefaultWebrtcConfiguration,
                DataChannelConfiguration::create());
        }

        m_client1->setTlsVerificationEnabled(false);
        m_client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    }

    void TearDown() override { m_client1->closeSync(); }
};

class WrongPasswordDataChannelClientTests : public DataChannelClientTests
{
protected:
    unique_ptr<DataChannelClient> m_client1;

    void SetUp() override
    {
        DataChannelClientTests::SetUp();

        m_client1 = make_unique<DataChannelClient>(
            SignalingServerConfiguration::createWithData(m_baseUrl, "c1", "cd1", "chat", ""),
            DefaultWebrtcConfiguration,
            DataChannelConfiguration::create());

        m_client1->setTlsVerificationEnabled(false);
        m_client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    }

    void TearDown() override
    {
        m_client1->closeSync();

        DataChannelClientTests::TearDown();
    }
};

class SingleDataChannelClientTests : public DataChannelClientTests
{
protected:
    unique_ptr<DataChannelClient> m_client1;

    void SetUp() override
    {
        DataChannelClientTests::SetUp();

        m_client1 = make_unique<DataChannelClient>(
            SignalingServerConfiguration::createWithData(m_baseUrl, "c1", "cd1", "chat", "abc"),
            DefaultWebrtcConfiguration,
            DataChannelConfiguration::create());

        m_client1->setTlsVerificationEnabled(false);
        m_client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    }

    void TearDown() override
    {
        m_client1->closeSync();

        DataChannelClientTests::TearDown();
    }
};

class RightPasswordDataChannelClientTests : public DataChannelClientTests
{
protected:
    unique_ptr<DataChannelClient> m_client1;
    unique_ptr<DataChannelClient> m_client2;
    unique_ptr<DataChannelClient> m_client3;

    string m_clientId1;
    string m_clientId2;
    string m_clientId3;

    void SetUp() override
    {
        DataChannelClientTests::SetUp();

        CallbackAwaiter setupAwaiter(3, 15s);
        m_client1 = make_unique<DataChannelClient>(
            SignalingServerConfiguration::createWithData(m_baseUrl, "c1", "cd1", "chat", "abc"),
            DefaultWebrtcConfiguration,
            DataChannelConfiguration::create());
        m_client2 = make_unique<DataChannelClient>(
            SignalingServerConfiguration::createWithData(m_baseUrl, "c2", "cd2", "chat", "abc"),
            DefaultWebrtcConfiguration,
            DataChannelConfiguration::create());
        m_client3 = make_unique<DataChannelClient>(
            SignalingServerConfiguration::createWithData(m_baseUrl, "c3", "cd3", "chat", "abc"),
            DefaultWebrtcConfiguration,
            DataChannelConfiguration::create());

        m_client1->setTlsVerificationEnabled(false);
        m_client2->setTlsVerificationEnabled(false);
        m_client3->setTlsVerificationEnabled(false);

        m_client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
        m_client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
        m_client3->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

        m_client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
        m_client2->setOnError([](const string& error) { ADD_FAILURE() << error; });
        m_client3->setOnError([](const string& error) { ADD_FAILURE() << error; });

        m_client1->connect();
        this_thread::sleep_for(250ms);
        m_client2->connect();
        this_thread::sleep_for(250ms);
        m_client3->connect();
        setupAwaiter.wait(__FILE__, __LINE__);

        m_client1->setOnSignalingConnectionOpened([] {});
        m_client2->setOnSignalingConnectionOpened([] {});
        m_client3->setOnSignalingConnectionOpened([] {});

        m_clientId1 = m_client1->id();
        m_clientId2 = m_client2->id();
        m_clientId3 = m_client3->id();
    }

    void TearDown() override
    {
        m_client1->closeSync();
        m_client2->closeSync();
        m_client3->closeSync();

        DataChannelClientTests::TearDown();
    }
};

TEST_P(DisconnectedDataChannelClientTests, isConnected_shouldReturnFalse)
{
    EXPECT_FALSE(m_client1->isConnected());
}

TEST_P(DisconnectedDataChannelClientTests, isRtcConnected_shouldReturnFalse)
{
    EXPECT_FALSE(m_client1->isRtcConnected());
}

TEST_P(DisconnectedDataChannelClientTests, id_shouldReturnAnEmptyString)
{
    EXPECT_EQ(m_client1->id(), "");
}

TEST_P(DisconnectedDataChannelClientTests, getConnectedRoomClientIds_shouldReturnAnEmptyVector)
{
    EXPECT_EQ(m_client1->getConnectedRoomClientIds().size(), 0);
}

TEST_P(DisconnectedDataChannelClientTests, getRoomClients_shouldReturnAnEmptyVector)
{
    EXPECT_EQ(m_client1->getRoomClients().size(), 0);
}

TEST_P(WrongPasswordDataChannelClientTests, connect_shouldGenerateAnError)
{
    CallbackAwaiter awaiter(2, 15s);
    m_client1->setOnSignalingConnectionOpened(
        [&]
        {
            ADD_FAILURE();
            awaiter.done();
            awaiter.done();
        });
    m_client1->setOnSignalingConnectionError(
        [&](const string& error)
        {
            EXPECT_EQ(m_client1->isConnected(), false);
            EXPECT_EQ(m_client1->isRtcConnected(), false);
            EXPECT_EQ(m_client1->id(), "");
            EXPECT_EQ(m_client1->getConnectedRoomClientIds().size(), 0);
            EXPECT_EQ(m_client1->getRoomClients().size(), 0);
            awaiter.done();
        });
    m_client1->setOnSignalingConnectionClosed([&] { awaiter.done(); });

    m_client1->connect();
    awaiter.wait(__FILE__, __LINE__);

    m_client1->setOnSignalingConnectionOpened([] {});
    m_client1->setOnSignalingConnectionError([](const string& error) {});
    m_client1->setOnSignalingConnectionClosed([] {});
}

TEST_P(SingleDataChannelClientTests, onRoomClientsChanged_shouldBeCallAfterTheConnection)
{
    CallbackAwaiter awaiter(2, 15s);
    m_client1->setOnSignalingConnectionOpened([&] { awaiter.done(); });
    m_client1->setOnRoomClientsChanged(
        [&](const vector<RoomClient>& roomClients)
        {
            EXPECT_EQ(roomClients.size(), 1);
            EXPECT_EQ(count(roomClients.begin(), roomClients.end(), RoomClient(m_client1->id(), "c1", "cd1", true)), 1);
            awaiter.done();
        });

    m_client1->connect();
    awaiter.wait(__FILE__, __LINE__);

    m_client1->setOnSignalingConnectionOpened([] {});
    m_client1->setOnRoomClientsChanged([](const vector<RoomClient>& roomClients) {});
}

TEST_P(RightPasswordDataChannelClientTests, isConnected_shouldReturnTrue)
{
    EXPECT_TRUE(m_client1->isConnected());
    EXPECT_TRUE(m_client2->isConnected());
    EXPECT_TRUE(m_client3->isConnected());
}

TEST_P(RightPasswordDataChannelClientTests, isRtcConnected_shouldReturnFalse)
{
    EXPECT_FALSE(m_client1->isRtcConnected());
    EXPECT_FALSE(m_client2->isRtcConnected());
    EXPECT_FALSE(m_client3->isRtcConnected());
}

TEST_P(RightPasswordDataChannelClientTests, id_shouldNotReturnAnEmptyString)
{
    EXPECT_NE(m_client1->id(), "");
    EXPECT_NE(m_client2->id(), "");
    EXPECT_NE(m_client3->id(), "");
}

TEST_P(RightPasswordDataChannelClientTests, getConnectedRoomClientIds_shouldNotReturnAnEmptyVector)
{
    EXPECT_EQ(m_client1->getConnectedRoomClientIds().size(), 0);
    EXPECT_EQ(m_client2->getConnectedRoomClientIds().size(), 0);
    EXPECT_EQ(m_client3->getConnectedRoomClientIds().size(), 0);
}

TEST_P(RightPasswordDataChannelClientTests, getRoomClient_shouldReturnTheSpecifiedClientOrDefault)
{
    EXPECT_EQ(m_client1->getRoomClient(m_client1->id()), RoomClient(m_client1->id(), "c1", "cd1", true));
    EXPECT_EQ(m_client1->getRoomClient(m_client2->id()), RoomClient(m_client2->id(), "c2", "cd2", false));
    EXPECT_EQ(m_client1->getRoomClient(m_client3->id()), RoomClient(m_client3->id(), "c3", "cd3", false));

    EXPECT_EQ(m_client1->getRoomClient(""), RoomClient());
}

TEST_P(RightPasswordDataChannelClientTests, getRoomClients_shouldReturnAllClients)
{
    auto roomClients1 = m_client1->getRoomClients();
    ASSERT_EQ(roomClients1.size(), 3);
    EXPECT_EQ(count(roomClients1.begin(), roomClients1.end(), RoomClient(m_client1->id(), "c1", "cd1", true)), 1);
    EXPECT_EQ(count(roomClients1.begin(), roomClients1.end(), RoomClient(m_client2->id(), "c2", "cd2", false)), 1);
    EXPECT_EQ(count(roomClients1.begin(), roomClients1.end(), RoomClient(m_client3->id(), "c3", "cd3", false)), 1);

    auto roomClients2 = m_client2->getRoomClients();
    ASSERT_EQ(roomClients2.size(), 3);
    EXPECT_EQ(count(roomClients2.begin(), roomClients2.end(), RoomClient(m_client1->id(), "c1", "cd1", false)), 1);
    EXPECT_EQ(count(roomClients2.begin(), roomClients2.end(), RoomClient(m_client2->id(), "c2", "cd2", true)), 1);
    EXPECT_EQ(count(roomClients2.begin(), roomClients2.end(), RoomClient(m_client3->id(), "c3", "cd3", false)), 1);

    auto roomClients3 = m_client3->getRoomClients();
    ASSERT_EQ(roomClients3.size(), 3);
    EXPECT_EQ(count(roomClients3.begin(), roomClients3.end(), RoomClient(m_client1->id(), "c1", "cd1", false)), 1);
    EXPECT_EQ(count(roomClients3.begin(), roomClients3.end(), RoomClient(m_client2->id(), "c2", "cd2", false)), 1);
    EXPECT_EQ(count(roomClients3.begin(), roomClients3.end(), RoomClient(m_client3->id(), "c3", "cd3", true)), 1);
}

TEST_P(RightPasswordDataChannelClientTests, callAll_shouldCallAllClients)
{
    CallbackAwaiter awaiter1(2, 60s);
    CallbackAwaiter awaiter2(2, 60s);
    CallbackAwaiter awaiter3(2, 60s);

    m_client1->setOnDataChannelOpened(
        [this, &awaiter1](const Client& client)
        {
            if (awaiter1.done())
            {
                EXPECT_TRUE(m_client1->isRtcConnected());
                auto connectedRoomClientIds = m_client1->getConnectedRoomClientIds();
                ASSERT_EQ(connectedRoomClientIds.size(), 2);
                EXPECT_EQ(count(connectedRoomClientIds.begin(), connectedRoomClientIds.end(), m_clientId2), 1);
                EXPECT_EQ(count(connectedRoomClientIds.begin(), connectedRoomClientIds.end(), m_clientId3), 1);
            }
        });
    m_client2->setOnDataChannelOpened(
        [this, &awaiter2](const Client& client)
        {
            if (awaiter2.done())
            {
                EXPECT_TRUE(m_client2->isRtcConnected());
                auto connectedRoomClientIds = m_client2->getConnectedRoomClientIds();
                ASSERT_EQ(connectedRoomClientIds.size(), 2);
                EXPECT_EQ(count(connectedRoomClientIds.begin(), connectedRoomClientIds.end(), m_clientId1), 1);
                EXPECT_EQ(count(connectedRoomClientIds.begin(), connectedRoomClientIds.end(), m_clientId3), 1);
            }
        });
    m_client3->setOnDataChannelOpened(
        [this, &awaiter3](const Client& client)
        {
            if (awaiter3.done())
            {
                EXPECT_TRUE(m_client3->isRtcConnected());
                auto connectedRoomClientIds = m_client3->getConnectedRoomClientIds();
                ASSERT_EQ(connectedRoomClientIds.size(), 2);
                EXPECT_EQ(count(connectedRoomClientIds.begin(), connectedRoomClientIds.end(), m_clientId1), 1);
                EXPECT_EQ(count(connectedRoomClientIds.begin(), connectedRoomClientIds.end(), m_clientId2), 1);
            }
        });

    m_client1->callAll();
    awaiter1.wait(__FILE__, __LINE__);
    awaiter2.wait(__FILE__, __LINE__);
    awaiter3.wait(__FILE__, __LINE__);

    m_client1->setOnDataChannelOpened([](const Client& client) {});
    m_client2->setOnDataChannelOpened([](const Client& client) {});
}

TEST_P(RightPasswordDataChannelClientTests, callIds_shouldCallTheSpecifiedClient)
{
    CallbackAwaiter awaiter(2, 60s);

    m_client1->setOnDataChannelOpened(
        [this, &awaiter](const Client& client)
        {
            awaiter.done();

            EXPECT_EQ(client.id(), m_clientId2);
            EXPECT_EQ(client.name(), "c2");
            EXPECT_EQ(client.data(), "cd2");
        });
    m_client2->setOnDataChannelOpened(
        [this, &awaiter](const Client& client)
        {
            awaiter.done();

            EXPECT_EQ(client.id(), m_clientId1);
            EXPECT_EQ(client.name(), "c1");
            EXPECT_EQ(client.data(), "cd1");
        });
    m_client3->setOnDataChannelOpened([](const Client& client) { ADD_FAILURE(); });

    m_client1->callIds({m_clientId2});
    awaiter.wait(__FILE__, __LINE__);

    m_client1->setOnDataChannelOpened([](const Client& client) {});
    m_client2->setOnDataChannelOpened([](const Client& client) {});
    m_client3->setOnDataChannelOpened([](const Client& client) {});
}

TEST_P(RightPasswordDataChannelClientTests, onClientConnected_shouldBeCalledAfterACall)
{
    CallbackAwaiter awaiter(1, 60s);
    m_client1->setOnClientConnected(
        [this, &awaiter](const Client& client)
        {
            awaiter.done();

            EXPECT_EQ(client.id(), m_clientId2);
            EXPECT_EQ(client.name(), "c2");
            EXPECT_EQ(client.data(), "cd2");
        });
    m_client2->setOnClientConnected(
        [this, &awaiter](const Client& client)
        {
            awaiter.done();

            EXPECT_EQ(client.id(), m_clientId1);
            EXPECT_EQ(client.name(), "c1");
            EXPECT_EQ(client.data(), "cd1");
        });
    m_client3->setOnClientConnected([](const Client& client) { ADD_FAILURE(); });

    m_client1->callIds({m_clientId2});
    awaiter.wait(__FILE__, __LINE__);

    m_client1->setOnClientConnected([](const Client& client) {});
    m_client2->setOnClientConnected([](const Client& client) {});
    m_client3->setOnClientConnected([](const Client& client) {});
}

TEST_P(RightPasswordDataChannelClientTests, onClientDisconnected_shouldBeCalledAfterHangUpAllCall)
{
    CallbackAwaiter awaiter(2, 60s);
    m_client1->setOnDataChannelOpened([this](const Client& client) { m_client1->hangUpAll(); });
    m_client1->setOnClientDisconnected(
        [this, &awaiter](const Client& client)
        {
            awaiter.done();

            EXPECT_EQ(client.id(), m_clientId2);
            EXPECT_EQ(client.name(), "c2");
            EXPECT_EQ(client.data(), "cd2");
        });
    m_client2->setOnClientDisconnected(
        [this, &awaiter](const Client& client)
        {
            awaiter.done();

            EXPECT_EQ(client.id(), m_clientId1);
            EXPECT_EQ(client.name(), "c1");
            EXPECT_EQ(client.data(), "cd1");
        });
    m_client3->setOnClientDisconnected([](const Client& client) { ADD_FAILURE(); });

    m_client1->callIds({m_clientId2});
    awaiter.wait(__FILE__, __LINE__);

    m_client1->setOnClientConnected([](const Client& client) {});
    m_client2->setOnClientConnected([](const Client& client) {});
    m_client3->setOnClientConnected([](const Client& client) {});
}

TEST_P(RightPasswordDataChannelClientTests, callAcceptor_shouldBeAbleToRejectACallAndOnCallRejectedShouldBeCalled)
{
    CallbackAwaiter awaiter(4, 60s);
    auto onFinish = [this, &awaiter]()
    {
        if (awaiter.done())
        {
            EXPECT_TRUE(m_client1->isRtcConnected());
            auto connectedRoomClientIds = m_client1->getConnectedRoomClientIds();
            ASSERT_EQ(connectedRoomClientIds.size(), 1);
            EXPECT_EQ(count(connectedRoomClientIds.begin(), connectedRoomClientIds.end(), m_clientId2), 1);

            EXPECT_TRUE(m_client2->isRtcConnected());
            connectedRoomClientIds = m_client2->getConnectedRoomClientIds();
            ASSERT_EQ(connectedRoomClientIds.size(), 1);
            EXPECT_EQ(count(connectedRoomClientIds.begin(), connectedRoomClientIds.end(), m_clientId1), 1);

            EXPECT_FALSE(m_client3->isRtcConnected());
            EXPECT_EQ(m_client3->getConnectedRoomClientIds().size(), 0);
        }
    };

    m_client1->setOnClientConnected(
        [this, &onFinish](const Client& client)
        {
            EXPECT_EQ(client.id(), m_clientId2);
            EXPECT_EQ(client.name(), "c2");
            EXPECT_EQ(client.data(), "cd2");

            onFinish();
        });
    m_client2->setOnClientConnected(
        [this, &onFinish](const Client& client)
        {
            EXPECT_EQ(client.id(), m_clientId1);
            EXPECT_EQ(client.name(), "c1");
            EXPECT_EQ(client.data(), "cd1");

            onFinish();
        });
    m_client3->setOnClientConnected([](const Client& client) { ADD_FAILURE(); });

    m_client1->setCallAcceptor(
        [](const Client& client)
        {
            ADD_FAILURE();
            return true;
        });
    m_client2->setCallAcceptor(
        [this](const Client& client)
        {
            if (client.id() == m_clientId1)
            {
                EXPECT_EQ(client.name(), "c1");
                EXPECT_EQ(client.data(), "cd1");
            }
            else if (client.id() == m_clientId3)
            {
                EXPECT_EQ(client.name(), "c3");
                EXPECT_EQ(client.data(), "cd3");
            }
            else
            {
                ADD_FAILURE();
            }
            return client.id() == m_clientId1;
        });
    m_client3->setCallAcceptor(
        [this](const Client& client)
        {
            if (client.id() == m_clientId1)
            {
                EXPECT_EQ(client.name(), "c1");
                EXPECT_EQ(client.data(), "cd1");
            }
            else
            {
                ADD_FAILURE();
            }
            return client.id() == m_clientId2;
        });

    m_client1->setOnCallRejected(
        [this, &onFinish](const Client& client)
        {
            EXPECT_EQ(client.id(), m_clientId3);
            EXPECT_EQ(client.name(), "c3");
            EXPECT_EQ(client.data(), "cd3");

            onFinish();
        });
    m_client2->setOnCallRejected(
        [this, &onFinish](const Client& client)
        {
            EXPECT_EQ(client.id(), m_clientId3);
            EXPECT_EQ(client.name(), "c3");
            EXPECT_EQ(client.data(), "cd3");

            onFinish();
        });
    m_client3->setOnCallRejected([](const Client& client) { ADD_FAILURE(); });

    m_client1->callAll();
    awaiter.wait(__FILE__, __LINE__);

    m_client1->setOnClientConnected([](const Client& client) {});
    m_client2->setOnClientConnected([](const Client& client) {});
    m_client3->setOnClientConnected([](const Client& client) {});

    m_client1->setCallAcceptor([](const Client& client) { return true; });
    m_client2->setCallAcceptor([](const Client& client) { return true; });
    m_client3->setCallAcceptor([](const Client& client) { return true; });
}

TEST_P(RightPasswordDataChannelClientTests, hangUpAll_shouldHangUpAllClients)
{
    CallbackAwaiter onDataChannelOpenedAwaiter(6, 60s);
    CallbackAwaiter halfOnDataChannelClosedAwaiter(4, 60s);
    CallbackAwaiter onDataChannelClosedAwaiter(6, 60s);

    auto onDataChannelOpened = [this, &onDataChannelOpenedAwaiter](const Client& client)
    {
        if (onDataChannelOpenedAwaiter.done())
        {
            m_client1->hangUpAll();
        }
    };

    auto onDataChannelClosed =
        [this, &halfOnDataChannelClosedAwaiter, &onDataChannelClosedAwaiter](const Client& client)
    {
        if (halfOnDataChannelClosedAwaiter.done())
        {
            m_client2->hangUpAll();
        }
        onDataChannelClosedAwaiter.done();
    };

    m_client1->setOnDataChannelOpened(onDataChannelOpened);
    m_client2->setOnDataChannelOpened(onDataChannelOpened);
    m_client3->setOnDataChannelOpened(onDataChannelOpened);

    m_client1->setOnDataChannelClosed(onDataChannelClosed);
    m_client2->setOnDataChannelClosed(onDataChannelClosed);
    m_client3->setOnDataChannelClosed(onDataChannelClosed);

    m_client1->callAll();
    onDataChannelOpenedAwaiter.wait(__FILE__, __LINE__);
    halfOnDataChannelClosedAwaiter.wait(__FILE__, __LINE__);
    onDataChannelClosedAwaiter.wait(__FILE__, __LINE__);

    m_client1->setOnDataChannelOpened([](const Client& client) {});
    m_client2->setOnDataChannelOpened([](const Client& client) {});
    m_client3->setOnDataChannelOpened([](const Client& client) {});

    m_client1->setOnDataChannelClosed([](const Client& client) {});
    m_client2->setOnDataChannelClosed([](const Client& client) {});
    m_client3->setOnDataChannelClosed([](const Client& client) {});
}

TEST_P(RightPasswordDataChannelClientTests, closeAllRoomPeerConnections_shouldCloseAllRoomPeerConnections)
{
    CallbackAwaiter onDataChannelOpenedAwaiter(6, 60s);
    CallbackAwaiter onDataChannelClosedAwaiter(6, 60s);

    auto onDataChannelOpened = [this, &onDataChannelOpenedAwaiter](const Client& client)
    {
        if (onDataChannelOpenedAwaiter.done())
        {
            m_client1->closeAllRoomPeerConnections();
        }
    };

    auto onDataChannelClosed = [this, &onDataChannelClosedAwaiter](const Client& client)
    { onDataChannelClosedAwaiter.done(); };

    m_client1->setOnDataChannelOpened(onDataChannelOpened);
    m_client2->setOnDataChannelOpened(onDataChannelOpened);
    m_client3->setOnDataChannelOpened(onDataChannelOpened);

    m_client1->setOnDataChannelClosed(onDataChannelClosed);
    m_client2->setOnDataChannelClosed(onDataChannelClosed);
    m_client3->setOnDataChannelClosed(onDataChannelClosed);

    m_client1->callAll();
    onDataChannelOpenedAwaiter.wait(__FILE__, __LINE__);
    onDataChannelClosedAwaiter.wait(__FILE__, __LINE__);

    m_client1->setOnDataChannelOpened([](const Client& client) {});
    m_client2->setOnDataChannelOpened([](const Client& client) {});
    m_client3->setOnDataChannelOpened([](const Client& client) {});

    m_client1->setOnDataChannelClosed([](const Client& client) {});
    m_client2->setOnDataChannelClosed([](const Client& client) {});
    m_client3->setOnDataChannelClosed([](const Client& client) {});
}

TEST_P(RightPasswordDataChannelClientTests, sendTo_binary_shouldSendTheDataToTheSpecifiedClients)
{
    CallbackAwaiter onDataChannelOpenAwaiter(6, 60s);
    CallbackAwaiter onDataChannelMessageAwaiter(3, 60s);

    auto onDataChannelOpened = [this, &onDataChannelOpenAwaiter](const Client& client)
    {
        if (onDataChannelOpenAwaiter.done())
        {
            uint8_t data1 = 101;
            uint8_t data2 = 102;
            uint8_t data3 = 103;

            m_client1->sendTo(&data1, 1, {m_clientId2});
            m_client2->sendTo(&data2, 1, {m_clientId3});
            m_client3->sendTo(&data3, 1, {m_clientId1});
        }
    };

    m_client1->setOnDataChannelOpened(onDataChannelOpened);
    m_client2->setOnDataChannelOpened(onDataChannelOpened);
    m_client3->setOnDataChannelOpened(onDataChannelOpened);

    m_client1->setOnDataChannelMessageBinary(
        [this, &onDataChannelMessageAwaiter](const Client& client, const uint8_t* data, size_t size)
        {
            onDataChannelMessageAwaiter.done();

            EXPECT_EQ(client.id(), m_clientId3);
            EXPECT_EQ(client.name(), "c3");
            EXPECT_EQ(client.data(), "cd3");
            ASSERT_EQ(size, 1);
            EXPECT_EQ(data[0], 103);
        });
    m_client2->setOnDataChannelMessageBinary(
        [this, &onDataChannelMessageAwaiter](const Client& client, const uint8_t* data, size_t size)
        {
            onDataChannelMessageAwaiter.done();

            EXPECT_EQ(client.id(), m_clientId1);
            EXPECT_EQ(client.name(), "c1");
            EXPECT_EQ(client.data(), "cd1");
            ASSERT_EQ(size, 1);
            EXPECT_EQ(data[0], 101);
        });
    m_client3->setOnDataChannelMessageBinary(
        [this, &onDataChannelMessageAwaiter](const Client& client, const uint8_t* data, size_t size)
        {
            onDataChannelMessageAwaiter.done();

            EXPECT_EQ(client.id(), m_clientId2);
            EXPECT_EQ(client.name(), "c2");
            EXPECT_EQ(client.data(), "cd2");
            ASSERT_EQ(size, 1);
            EXPECT_EQ(data[0], 102);
        });

    m_client1->callAll();
    onDataChannelOpenAwaiter.wait(__FILE__, __LINE__);
    onDataChannelMessageAwaiter.wait(__FILE__, __LINE__);

    m_client1->setOnDataChannelOpened([](const Client& client) {});
    m_client2->setOnDataChannelOpened([](const Client& client) {});
    m_client3->setOnDataChannelOpened([](const Client& client) {});

    m_client1->setOnDataChannelMessageBinary([](const Client& client, const uint8_t* data, size_t size) {});
    m_client2->setOnDataChannelMessageBinary([](const Client& client, const uint8_t* data, size_t size) {});
    m_client3->setOnDataChannelMessageBinary([](const Client& client, const uint8_t* data, size_t size) {});
}

TEST_P(RightPasswordDataChannelClientTests, sendTo_string_shouldSendTheDataToTheSpecifiedClients)
{
    CallbackAwaiter onDataChannelOpenedAwaiter(6, 60s);
    CallbackAwaiter onDataChannelMessageAwaiter(3, 60s);

    auto onDataChannelOpened = [this, &onDataChannelOpenedAwaiter](const Client& client)
    {
        if (onDataChannelOpenedAwaiter.done())
        {
            m_client1->sendTo("data1", {m_clientId2});
            m_client2->sendTo("data2", {m_clientId3});
            m_client3->sendTo("data3", {m_clientId1});
        }
    };

    m_client1->setOnDataChannelOpened(onDataChannelOpened);
    m_client2->setOnDataChannelOpened(onDataChannelOpened);
    m_client3->setOnDataChannelOpened(onDataChannelOpened);

    m_client1->setOnDataChannelMessageString(
        [this, &onDataChannelMessageAwaiter](const Client& client, const string& data)
        {
            onDataChannelMessageAwaiter.done();

            EXPECT_EQ(client.id(), m_clientId3);
            EXPECT_EQ(client.name(), "c3");
            EXPECT_EQ(client.data(), "cd3");
            EXPECT_EQ(data, "data3");
        });
    m_client2->setOnDataChannelMessageString(
        [this, &onDataChannelMessageAwaiter](const Client& client, const string& data)
        {
            onDataChannelMessageAwaiter.done();

            EXPECT_EQ(client.id(), m_clientId1);
            EXPECT_EQ(client.name(), "c1");
            EXPECT_EQ(client.data(), "cd1");
            EXPECT_EQ(data, "data1");
        });
    m_client3->setOnDataChannelMessageString(
        [this, &onDataChannelMessageAwaiter](const Client& client, const string& data)
        {
            onDataChannelMessageAwaiter.done();

            EXPECT_EQ(client.id(), m_clientId2);
            EXPECT_EQ(client.name(), "c2");
            EXPECT_EQ(client.data(), "cd2");
            EXPECT_EQ(data, "data2");
        });

    m_client1->callAll();
    onDataChannelOpenedAwaiter.wait(__FILE__, __LINE__);
    onDataChannelMessageAwaiter.wait(__FILE__, __LINE__);

    m_client1->setOnDataChannelOpened([](const Client& client) {});
    m_client2->setOnDataChannelOpened([](const Client& client) {});
    m_client3->setOnDataChannelOpened([](const Client& client) {});

    m_client1->setOnDataChannelMessageString([](const Client& client, const string& data) {});
    m_client2->setOnDataChannelMessageString([](const Client& client, const string& data) {});
    m_client3->setOnDataChannelMessageString([](const Client& client, const string& data) {});
}

TEST_P(RightPasswordDataChannelClientTests, sendToAll_binary_shouldSendTheDataToAllClients)
{
    CallbackAwaiter onDataChannelOpenedAwaiter(6, 60s);
    CallbackAwaiter onDataChannelMessageAwaiter(6, 60s);

    auto onDataChannelOpened = [this, &onDataChannelOpenedAwaiter](const Client& client)
    {
        if (onDataChannelOpenedAwaiter.done())
        {
            uint8_t data1 = 101;
            uint8_t data2 = 102;
            uint8_t data3 = 103;

            m_client1->sendToAll(&data1, 1);
            m_client2->sendToAll(&data2, 1);
            m_client3->sendToAll(&data3, 1);
        }
    };

    m_client1->setOnDataChannelOpened(onDataChannelOpened);
    m_client2->setOnDataChannelOpened(onDataChannelOpened);
    m_client3->setOnDataChannelOpened(onDataChannelOpened);

    m_client1->setOnDataChannelMessageBinary(
        [this, &onDataChannelMessageAwaiter](const Client& client, const uint8_t* data, size_t size)
        {
            onDataChannelMessageAwaiter.done();

            if (client.id() == m_clientId2)
            {
                EXPECT_EQ(client.name(), "c2");
                EXPECT_EQ(client.data(), "cd2");
                EXPECT_EQ(size, 1);
                EXPECT_EQ(data[0], 102);
            }
            else if (client.id() == m_clientId3)
            {
                EXPECT_EQ(client.name(), "c3");
                EXPECT_EQ(client.data(), "cd3");
                ASSERT_EQ(size, 1);
                EXPECT_EQ(data[0], 103);
            }
            else
            {
                ADD_FAILURE();
            }
        });
    m_client2->setOnDataChannelMessageBinary(
        [this, &onDataChannelMessageAwaiter](const Client& client, const uint8_t* data, size_t size)
        {
            onDataChannelMessageAwaiter.done();

            if (client.id() == m_clientId1)
            {
                EXPECT_EQ(client.name(), "c1");
                EXPECT_EQ(client.data(), "cd1");
                ASSERT_EQ(size, 1);
                EXPECT_EQ(data[0], 101);
            }
            else if (client.id() == m_clientId3)
            {
                EXPECT_EQ(client.name(), "c3");
                EXPECT_EQ(client.data(), "cd3");
                ASSERT_EQ(size, 1);
                EXPECT_EQ(data[0], 103);
            }
            else
            {
                ADD_FAILURE();
            }
        });
    m_client3->setOnDataChannelMessageBinary(
        [this, &onDataChannelMessageAwaiter](const Client& client, const uint8_t* data, size_t size)
        {
            onDataChannelMessageAwaiter.done();

            if (client.id() == m_clientId1)
            {
                EXPECT_EQ(client.name(), "c1");
                EXPECT_EQ(client.data(), "cd1");
                ASSERT_EQ(size, 1);
                EXPECT_EQ(data[0], 101);
            }
            else if (client.id() == m_clientId2)
            {
                EXPECT_EQ(client.name(), "c2");
                EXPECT_EQ(client.data(), "cd2");
                ASSERT_EQ(size, 1);
                EXPECT_EQ(data[0], 102);
            }
            else
            {
                ADD_FAILURE();
            }
        });

    m_client1->callAll();
    onDataChannelOpenedAwaiter.wait(__FILE__, __LINE__);
    onDataChannelMessageAwaiter.wait(__FILE__, __LINE__);

    m_client1->setOnDataChannelOpened([](const Client& client) {});
    m_client2->setOnDataChannelOpened([](const Client& client) {});
    m_client3->setOnDataChannelOpened([](const Client& client) {});

    m_client1->setOnDataChannelMessageBinary([](const Client& client, const uint8_t* data, size_t size) {});
    m_client2->setOnDataChannelMessageBinary([](const Client& client, const uint8_t* data, size_t size) {});
    m_client3->setOnDataChannelMessageBinary([](const Client& client, const uint8_t* data, size_t size) {});
}

TEST_P(RightPasswordDataChannelClientTests, sendToAll_string_shouldSendTheDataToTheSpecifiedClients)
{
    CallbackAwaiter onDataChannelOpenedAwaiter(6, 60s);
    CallbackAwaiter onDataChannelMessageAwaiter(3, 60s);

    auto onDataChannelOpened = [this, &onDataChannelOpenedAwaiter](const Client& client)
    {
        if (onDataChannelOpenedAwaiter.done())
        {
            m_client1->sendToAll("data1");
            m_client2->sendToAll("data2");
            m_client3->sendToAll("data3");
        }
    };

    m_client1->setOnDataChannelOpened(onDataChannelOpened);
    m_client2->setOnDataChannelOpened(onDataChannelOpened);
    m_client3->setOnDataChannelOpened(onDataChannelOpened);

    m_client1->setOnDataChannelMessageString(
        [this, &onDataChannelMessageAwaiter](const Client& client, const string& data)
        {
            onDataChannelMessageAwaiter.done();

            if (client.id() == m_clientId2)
            {
                EXPECT_EQ(client.name(), "c2");
                EXPECT_EQ(client.data(), "cd2");
                EXPECT_EQ(data, "data2");
            }
            else if (client.id() == m_clientId3)
            {
                EXPECT_EQ(client.name(), "c3");
                EXPECT_EQ(client.data(), "cd3");
                EXPECT_EQ(data, "data3");
            }
            else
            {
                ADD_FAILURE();
            }
        });
    m_client2->setOnDataChannelMessageString(
        [this, &onDataChannelMessageAwaiter](const Client& client, const string& data)
        {
            onDataChannelMessageAwaiter.done();

            if (client.id() == m_clientId1)
            {
                EXPECT_EQ(client.name(), "c1");
                EXPECT_EQ(client.data(), "cd1");
                EXPECT_EQ(data, "data1");
            }
            else if (client.id() == m_clientId3)
            {
                EXPECT_EQ(client.name(), "c3");
                EXPECT_EQ(client.data(), "cd3");
                EXPECT_EQ(data, "data3");
            }
            else
            {
                ADD_FAILURE();
            }
        });
    m_client3->setOnDataChannelMessageString(
        [this, &onDataChannelMessageAwaiter](const Client& client, const string& data)
        {
            onDataChannelMessageAwaiter.done();

            if (client.id() == m_clientId1)
            {
                EXPECT_EQ(client.name(), "c1");
                EXPECT_EQ(client.data(), "cd1");
                EXPECT_EQ(data, "data1");
            }
            else if (client.id() == m_clientId2)
            {
                EXPECT_EQ(client.name(), "c2");
                EXPECT_EQ(client.data(), "cd2");
                EXPECT_EQ(data, "data2");
            }
            else
            {
                ADD_FAILURE();
            }
        });

    m_client1->callAll();
    onDataChannelOpenedAwaiter.wait(__FILE__, __LINE__);
    onDataChannelMessageAwaiter.wait(__FILE__, __LINE__);

    m_client1->setOnDataChannelOpened([](const Client& client) {});
    m_client2->setOnDataChannelOpened([](const Client& client) {});
    m_client3->setOnDataChannelOpened([](const Client& client) {});

    m_client1->setOnDataChannelMessageString([](const Client& client, const string& data) {});
    m_client2->setOnDataChannelMessageString([](const Client& client, const string& data) {});
    m_client3->setOnDataChannelMessageString([](const Client& client, const string& data) {});
}

INSTANTIATE_TEST_SUITE_P(
    WrongPasswordDataChannelClientTests,
    WrongPasswordDataChannelClientTests,
    ::testing::Values(false, true));

INSTANTIATE_TEST_SUITE_P(SingleDataChannelClientTests, SingleDataChannelClientTests, ::testing::Values(false, true));

INSTANTIATE_TEST_SUITE_P(
    RightPasswordDataChannelClientTests,
    RightPasswordDataChannelClientTests,
    ::testing::Values(false, true));

INSTANTIATE_TEST_SUITE_P(
    DisconnectedDataChannelClientTests,
    DisconnectedDataChannelClientTests,
    ::testing::Values(false, true));
