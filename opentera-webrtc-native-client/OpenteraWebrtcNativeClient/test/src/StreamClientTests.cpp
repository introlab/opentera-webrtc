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

constexpr int BitsPerSample = 16;
constexpr int SampleRate = 48000;
constexpr size_t NumberOfChannels = 1;
constexpr chrono::milliseconds SinAudioSourceFrameDuration = 10ms;

class SinAudioSource : public AudioSource
{
    uint16_t m_amplitude;
    atomic_bool m_stopped;
    thread m_thread;

public:
    explicit SinAudioSource(uint16_t amplitude) :
            AudioSource(AudioSourceConfiguration::create(false, false, false, false, false, false, false),
                BitsPerSample, SampleRate, NumberOfChannels),
            m_amplitude(amplitude),
            m_stopped(false),
            m_thread(&SinAudioSource::run, this)
    {
    }

    ~SinAudioSource() override
    {
        m_stopped.store(true);
        m_thread.join();
    }

private:
    void run()
    {
        vector<int16_t> data(SinAudioSourceFrameDuration.count() * SampleRate / 1000, 0);
        double t = 0;
        for (size_t i = 0; i < data.size(); i++)
        {
            data[i] = static_cast<int16_t>(m_amplitude * sin(t));
            t += 2 * M_PI / data.size();
        }

        while(!m_stopped.load())
        {
            sendFrame(data.data(), data.size() * sizeof(int16_t) / bytesPerFrame());

            this_thread::sleep_for(SinAudioSourceFrameDuration);
        }
    }
};

static const WebrtcConfiguration DefaultWebrtcConfiguration = WebrtcConfiguration::create(
{
    IceServer("stun:stun.l.google.com:19302")
});

class StreamClientTests : public ::testing::TestWithParam<bool>
{
    static unique_ptr<subprocess::Popen> m_signalingServerProcess;
    static unique_ptr<subprocess::Popen> m_signalingServerProcessTLS;

protected:

    bool m_tlsTestEnable;
    string m_baseUrl;

    static void SetUpTestSuite()
    {
        fs::path testFilePath(__FILE__);
        fs::path pythonFilePath = testFilePath.parent_path().parent_path().parent_path().parent_path().parent_path()
                / "signaling-server" / "signaling_server.py";

        m_signalingServerProcess = make_unique<subprocess::Popen>("python3 " + pythonFilePath.string() +
                " --port 8080 --password abc --socketio_path thepath", subprocess::input(subprocess::PIPE));

        m_signalingServerProcessTLS = make_unique<subprocess::Popen>("python3 " + pythonFilePath.string() +
                " --port 8081 --password abc --socketio_path thepath"
                " --certificate resources/cert.pem --key resources/key.pem", subprocess::input(subprocess::PIPE));

        this_thread::sleep_for(2s);
    }

    static void TearDownTestSuite()
    {
        if (m_signalingServerProcess)
        {
            m_signalingServerProcess->kill(9);
            m_signalingServerProcess->wait();
        }

        if (m_signalingServerProcessTLS)
        {
            m_signalingServerProcessTLS->kill(9);
            m_signalingServerProcessTLS->wait();
        }
    }

    void SetUp()
    {
        m_tlsTestEnable = GetParam();

        if(m_tlsTestEnable)
        {
            m_baseUrl = "https://localhost:8081/thepath";
        }
        else
        {
            m_baseUrl = "http://localhost:8080/thepath";
        }
    }

    void TearDown()
    {

    }
};
unique_ptr<subprocess::Popen> StreamClientTests::m_signalingServerProcess = nullptr;
unique_ptr<subprocess::Popen> StreamClientTests::m_signalingServerProcessTLS = nullptr;

TEST_P(StreamClientTests, videoStream_bidirectional_shouldBeSentAndReceived)
{
    // Initialize the clients
    shared_ptr<ConstantVideoSource> videoSource1 = make_shared<ConstantVideoSource>(cv::Scalar(0, 0, 255));
    shared_ptr<ConstantVideoSource> videoSource2 = make_shared<ConstantVideoSource>(cv::Scalar(255, 0, 0));

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
            SignalingServerConfiguration::create(m_baseUrl, "c1", sio::string_message::create("cd1"), "chat", "abc"),
            DefaultWebrtcConfiguration, videoSource1);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
            SignalingServerConfiguration::create(m_baseUrl, "c2", sio::string_message::create("cd2"), "chat", "abc"),
            DefaultWebrtcConfiguration, videoSource2);

    client1->setTlsVerificationEnabled(false);
    client2->setTlsVerificationEnabled(false);

    client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait();

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
    client2->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
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

TEST_P(StreamClientTests, videoStream_unidirectional_shouldBeSentAndReceived)
{
    // Initialize the clients
    shared_ptr<ConstantVideoSource> videoSource = make_shared<ConstantVideoSource>(cv::Scalar(0, 0, 255));

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
            SignalingServerConfiguration::create(m_baseUrl, "c1", sio::string_message::create("cd1"), "chat", "abc"),
            DefaultWebrtcConfiguration, videoSource);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
            SignalingServerConfiguration::create(m_baseUrl, "c2", sio::string_message::create("cd2"), "chat", "abc"),
            DefaultWebrtcConfiguration);

    client1->setTlsVerificationEnabled(false);
    client2->setTlsVerificationEnabled(false);

    client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait();

    // Setup the callback
    CallbackAwaiter onVideoFrameAwaiter(1, 15s);

    unique_ptr<Client> onAddRemoteStreamClient;

    client1->setOnAddRemoteStream([&](const Client&) { ADD_FAILURE(); });
    client1->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client1->setOnVideoFrameReceived([&](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client1->setOnAudioFrameReceived([](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    client2->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient = make_unique<Client>(c); });
    client2->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client2->setOnVideoFrameReceived([&](const Client&, const cv::Mat&, uint64_t)
    {
        onVideoFrameAwaiter.done();
    });
    client2->setOnAudioFrameReceived([](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    // Setup the call
    client1->callAll();
    onVideoFrameAwaiter.wait();
    client1->hangUpAll();

    client1->closeSync();
    client2->closeSync();

    // Asserts
    ASSERT_NE(onAddRemoteStreamClient, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient->name(), "c1");
}

TEST_P(StreamClientTests, audioStream_bidirectional_shouldBeSentAndReceived)
{
    // Initialize the clients
    constexpr int16_t Amplitude1 = 5000;
    constexpr int16_t Amplitude2 = 15000;
    shared_ptr<SinAudioSource> audioSource1 = make_shared<SinAudioSource>(Amplitude1);
    shared_ptr<SinAudioSource> audioSource2 = make_shared<SinAudioSource>(Amplitude2);

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
            SignalingServerConfiguration::create(m_baseUrl, "c1", sio::string_message::create("cd1"), "chat", "abc"),
            DefaultWebrtcConfiguration, audioSource1);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
            SignalingServerConfiguration::create(m_baseUrl, "c2", sio::string_message::create("cd2"), "chat", "abc"),
            DefaultWebrtcConfiguration, audioSource2);

    client1->setTlsVerificationEnabled(false);
    client2->setTlsVerificationEnabled(false);

    client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait();

    // Setup the callback
    CallbackAwaiter onAudioFrameAwaiter1(5, 15s);
    CallbackAwaiter onAudioFrameAwaiter2(5, 15s);

    unique_ptr<Client> onAddRemoteStreamClient1;
    unique_ptr<Client> onAddRemoteStreamClient2;
    vector<int16_t> receivedAudio1;
    vector<int16_t> receivedAudio2;

    client1->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient1 = make_unique<Client>(c); });
    client1->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client1->setOnVideoFrameReceived([](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client1->setOnAudioFrameReceived([&](const Client& client,
        const void* audioData,
        int bitsPerSample,
        int sampleRate,
        size_t numberOfChannels,
        size_t numberOfFrames)
    {
        if (bitsPerSample != 16)
        {
            ADD_FAILURE();
        }
        else
        {
            const int16_t* begin = reinterpret_cast<const int16_t*>(audioData);
            const int16_t* end = begin + numberOfChannels * numberOfFrames;
            receivedAudio1.insert(receivedAudio1.begin(), begin, end);
        }
        onAudioFrameAwaiter1.done();
    });

    client2->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient2 = make_unique<Client>(c); });
    client2->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client2->setOnVideoFrameReceived([](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client2->setOnAudioFrameReceived([&](const Client& client,
         const void* audioData,
         int bitsPerSample,
         int sampleRate,
         size_t numberOfChannels,
         size_t numberOfFrames)
    {
        if (bitsPerSample != 16)
        {
            ADD_FAILURE();
        }
        else
        {
            const int16_t* begin = reinterpret_cast<const int16_t*>(audioData);
            const int16_t* end = begin + numberOfChannels * numberOfFrames;
            receivedAudio2.insert(receivedAudio2.begin(), begin, end);
        }
        onAudioFrameAwaiter2.done();
    });

    // Setup the call
    client1->callAll();
    onAudioFrameAwaiter1.wait();
    onAudioFrameAwaiter2.wait();
    client1->hangUpAll();
    this_thread::sleep_for(250ms);

    client1->closeSync();
    client2->closeSync();

    // Asserts
    ASSERT_NE(onAddRemoteStreamClient1, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient1->name(), "c2");
    ASSERT_NE(onAddRemoteStreamClient2, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient2->name(), "c1");

    constexpr int AbsError = 1000;

    ASSERT_FALSE(receivedAudio1.empty());
    int16_t min1 = *min_element(receivedAudio1.begin(), receivedAudio1.end());
    int16_t max1 = *max_element(receivedAudio1.begin(), receivedAudio1.end());
    EXPECT_NEAR(min1, -Amplitude2, AbsError);
    EXPECT_NEAR(max1, Amplitude2, AbsError);

    ASSERT_FALSE(receivedAudio2.empty());
    int16_t min2 = *min_element(receivedAudio2.begin(), receivedAudio2.end());
    int16_t max2 = *max_element(receivedAudio2.begin(), receivedAudio2.end());
    EXPECT_NEAR(min2, -Amplitude1, AbsError);
    EXPECT_NEAR(max2, Amplitude1, AbsError);
}

TEST_P(StreamClientTests, audioStream_unidirectional_shouldBeSentAndReceived)
{
    // Initialize the clients
    constexpr int16_t Amplitude = 5000;
    shared_ptr<SinAudioSource> audioSource = make_shared<SinAudioSource>(Amplitude);

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
            SignalingServerConfiguration::create(m_baseUrl, "c1", sio::string_message::create("cd1"), "chat", "abc"),
            DefaultWebrtcConfiguration, audioSource);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
            SignalingServerConfiguration::create(m_baseUrl, "c2", sio::string_message::create("cd2"), "chat", "abc"),
            DefaultWebrtcConfiguration);

    client1->setTlsVerificationEnabled(false);
    client2->setTlsVerificationEnabled(false);

    client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait();

    // Setup the callback
    CallbackAwaiter onAudioFrameAwaiter(5, 15s);

    unique_ptr<Client> onAddRemoteStreamClient;

    client1->setOnAddRemoteStream([&](const Client&) { ADD_FAILURE(); });
    client1->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client1->setOnVideoFrameReceived([](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client1->setOnAudioFrameReceived([&](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    client2->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient = make_unique<Client>(c); });
    client2->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client2->setOnVideoFrameReceived([](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client2->setOnAudioFrameReceived([&](const Client&, const void*, int, int, size_t, size_t)
    {
        onAudioFrameAwaiter.done();
    });
    // TODO setOnMixedAudioFrameReceived

    // Setup the call
    client1->callAll();
    onAudioFrameAwaiter.wait();
    client1->hangUpAll();

    client1->closeSync();
    client2->closeSync();

    // Asserts
    ASSERT_NE(onAddRemoteStreamClient, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient->name(), "c1");
}

INSTANTIATE_TEST_SUITE_P(
        StreamClientTests,
        StreamClientTests,
        ::testing::Values(
                false, true
        ));
