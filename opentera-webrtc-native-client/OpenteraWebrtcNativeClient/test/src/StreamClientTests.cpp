#include <OpenteraWebrtcNativeClient/StreamClient.h>

#include <OpenteraWebrtcNativeClientTests/CallbackAwaiter.h>

#include <gtest/gtest.h>

#include <subprocess.hpp>

#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

using namespace opentera;
using namespace std;
namespace fs = boost::filesystem;

class ConstantVideoSource : public VideoSource
{
    atomic_bool m_stopped;
    thread m_thread;

    cv::Scalar m_color;

public:
    explicit ConstantVideoSource(cv::Scalar color) :
            VideoSource(VideoSourceConfiguration::create(false, true)),
            m_stopped(false),
            m_thread(&ConstantVideoSource::run, this),
            m_color(move(color))
    {
    }

    ~ConstantVideoSource() override
    {
        m_stopped.store(true);
        m_thread.join();
    }

private:
    void run()
    {
        cv::Mat bgrImg(480, 640, CV_8UC3, m_color);
        while(!m_stopped.load())
        {
            this_thread::sleep_for(100ms);
            int64_t timestampUs =
                    chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now().time_since_epoch()).count();
            sendFrame(bgrImg, timestampUs);
        }
    }
};

static const WebrtcConfiguration DefaultWebrtcConfiguration = WebrtcConfiguration::create(
{
    IceServer("stun:stun.l.google.com:19302")
});

class StreamClientTests : public ::testing::Test
{
    static unique_ptr<subprocess::Popen> m_signalingServerProcess;

protected:
    static void SetUpTestSuite()
    {
        fs::path testFilePath(__FILE__);
        fs::path pythonFilePath = testFilePath.parent_path().parent_path().parent_path().parent_path().parent_path()
                / "signaling-server" / "signaling_server.py";
        m_signalingServerProcess = make_unique<subprocess::Popen>("python3 " + pythonFilePath.string() +
                " --port 8080 --password abc", subprocess::input(subprocess::PIPE));
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
unique_ptr<subprocess::Popen> StreamClientTests::m_signalingServerProcess = nullptr;

TEST_F(StreamClientTests, videoStream_shouldBeSentAndReceived)
{
    // Initialize the clients
    shared_ptr<ConstantVideoSource> videoSource1 = make_shared<ConstantVideoSource>(cv::Scalar(0, 0, 255));
    shared_ptr<ConstantVideoSource> videoSource2 = make_shared<ConstantVideoSource>(cv::Scalar(255, 0, 0));

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
            SignalingServerConfiguration::create("http://localhost:8080", "c1",
                    sio::string_message::create("cd1"), "chat", "abc"),
            DefaultWebrtcConfiguration, videoSource1);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
            SignalingServerConfiguration::create("http://localhost:8080", "c2",
                    sio::string_message::create("cd2"), "chat", "abc"),
            DefaultWebrtcConfiguration, videoSource2);

    client1->setOnSignalingConnectionOpen([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpen([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait();

    client1->setOnSignalingConnectionOpen([] {});
    client2->setOnSignalingConnectionOpen([] {});

    // Setup the callback
    CallbackAwaiter onVideoFrameAwaiter1(1, 15s);
    CallbackAwaiter onVideoFrameAwaiter2(1, 15s);

    unique_ptr<Client> onAddRemoteStreamClient1;
    unique_ptr<Client> onAddRemoteStreamClient2;
    cv::Mat receivedBgrImage1;
    cv::Mat receivedBgrImage2;

    client1->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient1 = make_unique<Client>(c); });
    client1->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client1->setOnVideoFrameReceived([&](const Client&, const cv::Mat& bgrImg, uint64_t)
    {
        receivedBgrImage1 = bgrImg.clone();
        onVideoFrameAwaiter1.done();
    });
    client1->setOnAudioFrameReceived([](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    client2->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient2 = make_unique<Client>(c); });
    client2->setOnRemoveRemoteStream([](const Client& c) { ADD_FAILURE(); });
    client2->setOnVideoFrameReceived([&](const Client&, const cv::Mat& bgrImg, uint64_t)
    {
        receivedBgrImage2 = bgrImg.clone();
        onVideoFrameAwaiter2.done();
    });
    client2->setOnAudioFrameReceived([](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    // Setup the call
    client1->callAll();
    onVideoFrameAwaiter1.wait();
    onVideoFrameAwaiter2.wait();
    client1->hangUpAll();
    this_thread::sleep_for(250ms);

    client1->closeSync();
    client2->closeSync();

    // Asserts
    ASSERT_NE(onAddRemoteStreamClient1, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient1->name(), "c2");
    ASSERT_NE(onAddRemoteStreamClient2, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient2->name(), "c1");

    constexpr int MeanColorAbsError = 15;

    cv::Scalar meanColor1 = cv::mean(receivedBgrImage1);
    EXPECT_NEAR(meanColor1[0], 255, MeanColorAbsError);
    EXPECT_NEAR(meanColor1[1], 0, MeanColorAbsError);
    EXPECT_NEAR(meanColor1[2], 0, MeanColorAbsError);

    cv::Scalar meanColor2 = cv::mean(receivedBgrImage2);
    EXPECT_NEAR(meanColor2[0], 0, MeanColorAbsError);
    EXPECT_NEAR(meanColor2[1], 0, MeanColorAbsError);
    EXPECT_NEAR(meanColor2[2], 255, MeanColorAbsError);
}
