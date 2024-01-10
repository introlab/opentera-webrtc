#include <OpenteraWebrtcNativeClient/StreamClient.h>

#include <OpenteraWebrtcNativeClientTests/CallbackAwaiter.h>

#include <gtest/gtest.h>

#include <subprocess.hpp>

#include <filesystem>

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

using namespace opentera;
using namespace std;
namespace fs = std::filesystem;

class ConstantVideoSource : public VideoSource
{
    atomic_bool m_stopped;
    thread m_thread;

    cv::Scalar m_color;

public:
    explicit ConstantVideoSource(cv::Scalar color)
        : VideoSource(VideoSourceConfiguration::create(false, true)),
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
        while (!m_stopped.load())
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
    explicit SinAudioSource(uint16_t amplitude)
        : AudioSource(
              AudioSourceConfiguration::create(0, false, false, false, false, false, false),
              BitsPerSample,
              SampleRate,
              NumberOfChannels),
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

        while (!m_stopped.load())
        {
            sendFrame(data.data(), data.size() * sizeof(int16_t) / bytesPerFrame());

            this_thread::sleep_for(SinAudioSourceFrameDuration);
        }
    }
};

static const WebrtcConfiguration DefaultWebrtcConfiguration =
    WebrtcConfiguration::create({IceServer("stun:stun.l.google.com:19302")});

struct StreamClientTestsParameters
{
    bool tlsTestEnable;
    bool useGStreamerSoftwareEncoderDecoder;
};

void PrintTo(const StreamClientTestsParameters& parameters, ostream* os)
{
    *os << "tlsTestEnable=" << parameters.tlsTestEnable;
    *os << ", useGStreamerSoftwareEncoderDecoder=" << parameters.useGStreamerSoftwareEncoderDecoder;
}

class StreamClientTests : public ::testing::TestWithParam<StreamClientTestsParameters>
{
    static unique_ptr<subprocess::Popen> m_signalingServerProcess;
    static unique_ptr<subprocess::Popen> m_signalingServerProcessTLS;

protected:
    VideoStreamConfiguration m_videoStreamConfiguration;
    string m_baseUrl;

    StreamClientTests() : m_videoStreamConfiguration(VideoStreamConfiguration::create()) {}

    static void SetUpTestSuite()
    {
        fs::path testFilePath(__FILE__);
        fs::path pythonFilePath = testFilePath.parent_path().parent_path().parent_path().parent_path().parent_path() /
                                  "signaling-server" / "opentera-signaling-server";

        m_signalingServerProcess = make_unique<subprocess::Popen>(
            "python3 " + pythonFilePath.string() + " --port 8080 --password abc",
            subprocess::input(subprocess::PIPE));

        m_signalingServerProcessTLS = make_unique<subprocess::Popen>(
            "python3 " + pythonFilePath.string() +
                " --port 8081 --password abc"
                " --certificate resources/cert.pem --key resources/key.pem",
            subprocess::input(subprocess::PIPE));

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

    void SetUp() override
    {
        StreamClientTestsParameters parameters = GetParam();
        m_videoStreamConfiguration =
            VideoStreamConfiguration::create({}, false, parameters.useGStreamerSoftwareEncoderDecoder);

        if (parameters.tlsTestEnable)
        {
            m_baseUrl = "wss://localhost:8081/signaling";
        }
        else
        {
            m_baseUrl = "ws://localhost:8080/signaling";
        }
    }
};
unique_ptr<subprocess::Popen> StreamClientTests::m_signalingServerProcess = nullptr;
unique_ptr<subprocess::Popen> StreamClientTests::m_signalingServerProcessTLS = nullptr;

TEST_P(StreamClientTests, muteMethods_shouldSetTheFlagAccordingly)
{
    unique_ptr<StreamClient> client = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c1", "cd1", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration);

    EXPECT_FALSE(client->isLocalAudioMuted());
    EXPECT_FALSE(client->isRemoteAudioMuted());
    EXPECT_FALSE(client->isLocalVideoMuted());

    client->muteLocalAudio();
    EXPECT_TRUE(client->isLocalAudioMuted());
    EXPECT_FALSE(client->isRemoteAudioMuted());
    EXPECT_FALSE(client->isLocalVideoMuted());

    client->muteRemoteAudio();
    EXPECT_TRUE(client->isLocalAudioMuted());
    EXPECT_TRUE(client->isRemoteAudioMuted());
    EXPECT_FALSE(client->isLocalVideoMuted());

    client->muteLocalVideo();
    EXPECT_TRUE(client->isLocalAudioMuted());
    EXPECT_TRUE(client->isRemoteAudioMuted());
    EXPECT_TRUE(client->isLocalVideoMuted());

    client->unmuteLocalAudio();
    EXPECT_FALSE(client->isLocalAudioMuted());
    EXPECT_TRUE(client->isRemoteAudioMuted());
    EXPECT_TRUE(client->isLocalVideoMuted());

    client->unmuteRemoteAudio();
    EXPECT_FALSE(client->isLocalAudioMuted());
    EXPECT_FALSE(client->isRemoteAudioMuted());
    EXPECT_TRUE(client->isLocalVideoMuted());

    client->unmuteLocalVideo();
    EXPECT_FALSE(client->isLocalAudioMuted());
    EXPECT_FALSE(client->isRemoteAudioMuted());
    EXPECT_FALSE(client->isLocalVideoMuted());

    client->setLocalAudioMuted(true);
    EXPECT_TRUE(client->isLocalAudioMuted());
    EXPECT_FALSE(client->isRemoteAudioMuted());
    EXPECT_FALSE(client->isLocalVideoMuted());

    client->setRemoteAudioMuted(true);
    EXPECT_TRUE(client->isLocalAudioMuted());
    EXPECT_TRUE(client->isRemoteAudioMuted());
    EXPECT_FALSE(client->isLocalVideoMuted());

    client->setLocalVideoMuted(true);
    EXPECT_TRUE(client->isLocalAudioMuted());
    EXPECT_TRUE(client->isRemoteAudioMuted());
    EXPECT_TRUE(client->isLocalVideoMuted());
}

TEST_P(StreamClientTests, videoStream_bidirectional_shouldBeSentAndReceived)
{
    // Initialize the clients
    shared_ptr<ConstantVideoSource> videoSource1 = make_shared<ConstantVideoSource>(cv::Scalar(0, 0, 255));
    shared_ptr<ConstantVideoSource> videoSource2 = make_shared<ConstantVideoSource>(cv::Scalar(255, 0, 0));

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c1", "cd1", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration,
        videoSource1);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c2", "cd2", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration,
        videoSource2);

    client1->setTlsVerificationEnabled(false);
    client2->setTlsVerificationEnabled(false);

    client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait(__FILE__, __LINE__);

    // Setup the callback
    CallbackAwaiter onVideoFrameAwaiter1(10, 15s);
    CallbackAwaiter onVideoFrameAwaiter2(10, 15s);
    CallbackAwaiter onEncodedVideoFrameAwaiter1(1, 15s);
    CallbackAwaiter onEncodedVideoFrameAwaiter2(1, 15s);

    unique_ptr<Client> onAddRemoteStreamClient1;
    unique_ptr<Client> onAddRemoteStreamClient2;
    cv::Mat receivedBgrImage1;
    cv::Mat receivedBgrImage2;

    client1->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient1 = make_unique<Client>(c); });
    client1->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client1->setOnVideoFrameReceived(
        [&](const Client& client, const cv::Mat& bgrImg, uint64_t)
        {
            EXPECT_EQ(client.name(), "c2");
            receivedBgrImage1 = bgrImg.clone();
            onVideoFrameAwaiter1.done();
        });
    client1->setOnEncodedVideoFrameReceived(
        [&](const Client& client, const uint8_t*, size_t, VideoCodecType, bool, uint32_t, uint32_t, uint64_t)
        {
            EXPECT_EQ(client.name(), "c2");
            onEncodedVideoFrameAwaiter1.done();
        });
    client1->setOnAudioFrameReceived([](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });
    client1->setOnMixedAudioFrameReceived([](const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    client2->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient2 = make_unique<Client>(c); });
    client2->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client2->setOnVideoFrameReceived(
        [&](const Client& client, const cv::Mat& bgrImg, uint64_t)
        {
            EXPECT_EQ(client.name(), "c1");
            receivedBgrImage2 = bgrImg.clone();
            onVideoFrameAwaiter2.done();
        });
    client2->setOnEncodedVideoFrameReceived(
        [&](const Client& client, const uint8_t*, size_t, VideoCodecType, bool, uint32_t, uint32_t, uint64_t)
        {
            EXPECT_EQ(client.name(), "c1");
            onEncodedVideoFrameAwaiter2.done();
        });
    client2->setOnAudioFrameReceived([](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });
    client2->setOnMixedAudioFrameReceived([](const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    // Setup the call
    client1->callAll();
    onVideoFrameAwaiter1.wait(__FILE__, __LINE__);
    onEncodedVideoFrameAwaiter1.wait(__FILE__, __LINE__);
    onVideoFrameAwaiter2.wait(__FILE__, __LINE__);
    onEncodedVideoFrameAwaiter2.wait(__FILE__, __LINE__);
    client1->hangUpAll();

    client1->closeSync();
    client2->closeSync();

    // Asserts
    ASSERT_NE(onAddRemoteStreamClient1, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient1->name(), "c2");
    ASSERT_NE(onAddRemoteStreamClient2, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient2->name(), "c1");

    constexpr int MeanColorAbsError = 20;

    cv::Scalar meanColor1 = cv::mean(receivedBgrImage1);
    EXPECT_NEAR(meanColor1[0], 255, MeanColorAbsError);
    EXPECT_NEAR(meanColor1[1], 0, MeanColorAbsError);
    EXPECT_NEAR(meanColor1[2], 0, MeanColorAbsError);

    cv::Scalar meanColor2 = cv::mean(receivedBgrImage2);
    EXPECT_NEAR(meanColor2[0], 0, MeanColorAbsError);
    EXPECT_NEAR(meanColor2[1], 0, MeanColorAbsError);
    EXPECT_NEAR(meanColor2[2], 255, MeanColorAbsError);
}

TEST_P(StreamClientTests, videoStream_muted_shouldBeSentAndReceived)
{
    // Initialize the clients
    shared_ptr<ConstantVideoSource> videoSource1 = make_shared<ConstantVideoSource>(cv::Scalar(0, 0, 255));
    shared_ptr<ConstantVideoSource> videoSource2 = make_shared<ConstantVideoSource>(cv::Scalar(255, 0, 0));

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c1", "cd1", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration,
        videoSource1);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c2", "cd2", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration,
        videoSource2);

    client1->setTlsVerificationEnabled(false);
    client2->setTlsVerificationEnabled(false);

    client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait(__FILE__, __LINE__);

    // Setup the callback
    CallbackAwaiter onVideoFrameAwaiter1(10, 15s);
    CallbackAwaiter onVideoFrameAwaiter2(10, 15s);

    unique_ptr<Client> onAddRemoteStreamClient1;
    unique_ptr<Client> onAddRemoteStreamClient2;
    cv::Mat receivedBgrImage1;
    cv::Mat receivedBgrImage2;

    client1->muteLocalVideo();  // Muted before the call
    client1->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient1 = make_unique<Client>(c); });
    client1->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client1->setOnVideoFrameReceived(
        [&](const Client&, const cv::Mat& bgrImg, uint64_t)
        {
            client2->muteLocalVideo();  // Muted during the call
            receivedBgrImage1 = bgrImg.clone();
            onVideoFrameAwaiter1.done();
        });
    client1->setOnAudioFrameReceived([](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });
    client1->setOnMixedAudioFrameReceived([](const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    client2->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient2 = make_unique<Client>(c); });
    client2->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client2->setOnVideoFrameReceived(
        [&](const Client&, const cv::Mat& bgrImg, uint64_t)
        {
            receivedBgrImage2 = bgrImg.clone();
            onVideoFrameAwaiter2.done();
        });
    client2->setOnAudioFrameReceived([](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });
    client2->setOnMixedAudioFrameReceived([](const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    // Setup the call
    client1->callAll();
    onVideoFrameAwaiter1.wait(__FILE__, __LINE__);
    onVideoFrameAwaiter2.wait(__FILE__, __LINE__);
    client1->hangUpAll();

    client1->closeSync();
    client2->closeSync();

    // Asserts
    ASSERT_NE(onAddRemoteStreamClient1, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient1->name(), "c2");
    ASSERT_NE(onAddRemoteStreamClient2, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient2->name(), "c1");

    constexpr int MeanColorAbsError = 20;

    cv::Scalar meanColor1 = cv::mean(receivedBgrImage1);
    EXPECT_NEAR(meanColor1[0], 0, MeanColorAbsError);
    EXPECT_NEAR(meanColor1[1], 0, MeanColorAbsError);
    EXPECT_NEAR(meanColor1[2], 0, MeanColorAbsError);

    cv::Scalar meanColor2 = cv::mean(receivedBgrImage2);
    EXPECT_NEAR(meanColor2[0], 0, MeanColorAbsError);
    EXPECT_NEAR(meanColor2[1], 0, MeanColorAbsError);
    EXPECT_NEAR(meanColor2[2], 0, MeanColorAbsError);
}

TEST_P(StreamClientTests, videoStream_unidirectional_shouldBeSentAndReceived)
{
    // Initialize the clients
    shared_ptr<ConstantVideoSource> videoSource = make_shared<ConstantVideoSource>(cv::Scalar(0, 0, 255));

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c1", "cd1", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration,
        videoSource);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c2", "cd2", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration);

    client1->setTlsVerificationEnabled(false);
    client2->setTlsVerificationEnabled(false);

    client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait(__FILE__, __LINE__);

    // Setup the callback
    CallbackAwaiter onVideoFrameAwaiter(10, 15s);
    CallbackAwaiter onEncodedVideoFrameAwaiter(1, 15s);

    unique_ptr<Client> onAddRemoteStreamClient;

    client1->setOnAddRemoteStream([&](const Client&) { ADD_FAILURE(); });
    client1->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client1->setOnVideoFrameReceived([&](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client1->setOnEncodedVideoFrameReceived(
        [](const Client&, const uint8_t*, size_t, VideoCodecType, bool, uint32_t, uint32_t, uint64_t)
        { ADD_FAILURE(); });
    client1->setOnAudioFrameReceived([](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });
    client1->setOnMixedAudioFrameReceived([](const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    client2->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient = make_unique<Client>(c); });
    client2->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client2->setOnVideoFrameReceived([&](const Client&, const cv::Mat&, uint64_t) { onVideoFrameAwaiter.done(); });
    client2->setOnEncodedVideoFrameReceived(
        [&](const Client&, const uint8_t*, size_t, VideoCodecType, bool, uint32_t, uint32_t, uint64_t)
        { onEncodedVideoFrameAwaiter.done(); });
    client2->setOnAudioFrameReceived([](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });
    client2->setOnMixedAudioFrameReceived([](const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    // Setup the call
    client1->callAll();
    onVideoFrameAwaiter.wait(__FILE__, __LINE__);
    onEncodedVideoFrameAwaiter.wait(__FILE__, __LINE__);
    client1->hangUpAll();

    client1->closeSync();
    client2->closeSync();

    // Asserts
    ASSERT_NE(onAddRemoteStreamClient, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient->name(), "c1");
}

void onAudioFrameReceived(
    const void* audioData,
    int bitsPerSample,
    int sampleRate,
    size_t numberOfChannels,
    size_t numberOfFrames,
    vector<int16_t>& receivedAudio,
    CallbackAwaiter& awaiter)
{
    if (bitsPerSample != 16)
    {
        ADD_FAILURE();
    }
    else
    {
        const int16_t* begin = reinterpret_cast<const int16_t*>(audioData);
        const int16_t* end = begin + numberOfChannels * numberOfFrames;
        receivedAudio.insert(receivedAudio.end(), begin, end);
    }
    awaiter.done();
}

void checkReceivedAudio(const vector<int16_t>& receivedAudio, int16_t amplitude)
{
    constexpr int AbsError = 9000;

    ASSERT_FALSE(receivedAudio.empty());
    int16_t min1 = *min_element(receivedAudio.begin() + receivedAudio.size() / 2, receivedAudio.end());
    int16_t max1 = *max_element(receivedAudio.begin() + receivedAudio.size() / 2, receivedAudio.end());
    EXPECT_NEAR(min1, -amplitude, AbsError);
    EXPECT_NEAR(max1, amplitude, AbsError);
}

TEST_P(StreamClientTests, audioStream_bidirectional_shouldBeSentAndReceived)
{
    // Initialize the clients
    constexpr int16_t Amplitude1 = 10000;
    constexpr int16_t Amplitude2 = 20000;
    shared_ptr<SinAudioSource> audioSource1 = make_shared<SinAudioSource>(Amplitude1);
    shared_ptr<SinAudioSource> audioSource2 = make_shared<SinAudioSource>(Amplitude2);

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c1", "cd1", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration,
        audioSource1);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c2", "cd2", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration,
        audioSource2);

    client1->setTlsVerificationEnabled(false);
    client2->setTlsVerificationEnabled(false);

    client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait(__FILE__, __LINE__);

    // Setup the callback
    CallbackAwaiter onAudioFrameAwaiter1(50, 15s);
    CallbackAwaiter onAudioFrameAwaiter2(50, 15s);
    CallbackAwaiter onMixedAudioFrameAwaiter1(50, 15s);
    CallbackAwaiter onMixedAudioFrameAwaiter2(50, 15s);

    unique_ptr<Client> onAddRemoteStreamClient1;
    unique_ptr<Client> onAddRemoteStreamClient2;
    vector<int16_t> receivedAudio1;
    vector<int16_t> receivedAudio2;
    vector<int16_t> receivedMixedAudio1;
    vector<int16_t> receivedMixedAudio2;

    client1->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient1 = make_unique<Client>(c); });
    client1->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client1->setOnVideoFrameReceived([](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client1->setOnEncodedVideoFrameReceived(
        [](const Client&, const uint8_t*, size_t, VideoCodecType, bool, uint32_t, uint32_t, uint64_t)
        { ADD_FAILURE(); });
    client1->setOnAudioFrameReceived(
        [&](const Client& client,
            const void* audioData,
            int bitsPerSample,
            int sampleRate,
            size_t numberOfChannels,
            size_t numberOfFrames)
        {
            EXPECT_EQ(client.name(), "c2");
            onAudioFrameReceived(
                audioData,
                bitsPerSample,
                sampleRate,
                numberOfChannels,
                numberOfFrames,
                receivedAudio1,
                onAudioFrameAwaiter1);
        });
    client1->setOnMixedAudioFrameReceived(
        [&](const void* audioData, int bitsPerSample, int sampleRate, size_t numberOfChannels, size_t numberOfFrames)
        {
            onAudioFrameReceived(
                audioData,
                bitsPerSample,
                sampleRate,
                numberOfChannels,
                numberOfFrames,
                receivedMixedAudio1,
                onMixedAudioFrameAwaiter1);
        });

    client2->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient2 = make_unique<Client>(c); });
    client2->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client2->setOnVideoFrameReceived([](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client2->setOnEncodedVideoFrameReceived(
        [](const Client&, const uint8_t*, size_t, VideoCodecType, bool, uint32_t, uint32_t, uint64_t)
        { ADD_FAILURE(); });
    client2->setOnAudioFrameReceived(
        [&](const Client& client,
            const void* audioData,
            int bitsPerSample,
            int sampleRate,
            size_t numberOfChannels,
            size_t numberOfFrames)
        {
            EXPECT_EQ(client.name(), "c1");
            onAudioFrameReceived(
                audioData,
                bitsPerSample,
                sampleRate,
                numberOfChannels,
                numberOfFrames,
                receivedAudio2,
                onAudioFrameAwaiter2);
        });
    client2->setOnMixedAudioFrameReceived(
        [&](const void* audioData, int bitsPerSample, int sampleRate, size_t numberOfChannels, size_t numberOfFrames)
        {
            onAudioFrameReceived(
                audioData,
                bitsPerSample,
                sampleRate,
                numberOfChannels,
                numberOfFrames,
                receivedMixedAudio2,
                onMixedAudioFrameAwaiter2);
        });

    // Setup the call
    client1->callAll();
    onAudioFrameAwaiter1.wait(__FILE__, __LINE__);
    onMixedAudioFrameAwaiter1.wait(__FILE__, __LINE__);
    onAudioFrameAwaiter2.wait(__FILE__, __LINE__);
    onMixedAudioFrameAwaiter2.wait(__FILE__, __LINE__);
    client1->hangUpAll();
    client2->hangUpAll();
    client1->setOnMixedAudioFrameReceived([](const void*, int, int, size_t, size_t) {});
    client2->setOnMixedAudioFrameReceived([](const void*, int, int, size_t, size_t) {});

    this_thread::sleep_for(250ms);
    client1->closeSync();
    client2->closeSync();

    // Asserts
    ASSERT_NE(onAddRemoteStreamClient1, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient1->name(), "c2");
    ASSERT_NE(onAddRemoteStreamClient2, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient2->name(), "c1");

    checkReceivedAudio(receivedAudio1, Amplitude2);
    checkReceivedAudio(receivedMixedAudio1, Amplitude2);
    checkReceivedAudio(receivedAudio2, Amplitude1);
    checkReceivedAudio(receivedMixedAudio2, Amplitude1);
}

TEST_P(StreamClientTests, audioStream_muted_shouldBeSentAndReceived)
{
    // Initialize the clients
    constexpr int16_t Amplitude1 = 10000;
    constexpr int16_t Amplitude2 = 25000;
    shared_ptr<SinAudioSource> audioSource1 = make_shared<SinAudioSource>(Amplitude1);
    shared_ptr<SinAudioSource> audioSource2 = make_shared<SinAudioSource>(Amplitude2);

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c1", "cd1", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration,
        audioSource1);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c2", "cd2", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration,
        audioSource2);

    client1->setTlsVerificationEnabled(false);
    client2->setTlsVerificationEnabled(false);

    client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait(__FILE__, __LINE__);

    // Setup the callback
    CallbackAwaiter onAudioFrameAwaiter1(50, 60s);
    CallbackAwaiter onAudioFrameAwaiter2(50, 60s);
    CallbackAwaiter onMixedAudioFrameAwaiter1(50, 60s);
    CallbackAwaiter onMixedAudioFrameAwaiter2(50, 60s);

    unique_ptr<Client> onAddRemoteStreamClient1;
    unique_ptr<Client> onAddRemoteStreamClient2;
    vector<int16_t> receivedAudio1;
    vector<int16_t> receivedAudio2;
    vector<int16_t> receivedMixedAudio1;
    vector<int16_t> receivedMixedAudio2;

    client1->muteLocalAudio();  // Muted before the call
    client1->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient1 = make_unique<Client>(c); });
    client1->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client1->setOnVideoFrameReceived([](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client1->setOnEncodedVideoFrameReceived(
        [](const Client&, const uint8_t*, size_t, VideoCodecType, bool, uint32_t, uint32_t, uint64_t)
        { ADD_FAILURE(); });
    client1->setOnAudioFrameReceived(
        [&](const Client& client,
            const void* audioData,
            int bitsPerSample,
            int sampleRate,
            size_t numberOfChannels,
            size_t numberOfFrames)
        {
            client2->muteLocalAudio();  // Muted during the call
            onAudioFrameReceived(
                audioData,
                bitsPerSample,
                sampleRate,
                numberOfChannels,
                numberOfFrames,
                receivedAudio1,
                onAudioFrameAwaiter1);
        });
    client1->setOnMixedAudioFrameReceived(
        [&](const void* audioData, int bitsPerSample, int sampleRate, size_t numberOfChannels, size_t numberOfFrames)
        {
            onAudioFrameReceived(
                audioData,
                bitsPerSample,
                sampleRate,
                numberOfChannels,
                numberOfFrames,
                receivedMixedAudio1,
                onMixedAudioFrameAwaiter1);
        });

    client2->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient2 = make_unique<Client>(c); });
    client2->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client2->setOnVideoFrameReceived([](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client2->setOnEncodedVideoFrameReceived(
        [](const Client&, const uint8_t*, size_t, VideoCodecType, bool, uint32_t, uint32_t, uint64_t)
        { ADD_FAILURE(); });
    client2->setOnAudioFrameReceived(
        [&](const Client& client,
            const void* audioData,
            int bitsPerSample,
            int sampleRate,
            size_t numberOfChannels,
            size_t numberOfFrames)
        {
            onAudioFrameReceived(
                audioData,
                bitsPerSample,
                sampleRate,
                numberOfChannels,
                numberOfFrames,
                receivedAudio2,
                onAudioFrameAwaiter2);
        });
    client2->setOnMixedAudioFrameReceived(
        [&](const void* audioData, int bitsPerSample, int sampleRate, size_t numberOfChannels, size_t numberOfFrames)
        {
            onAudioFrameReceived(
                audioData,
                bitsPerSample,
                sampleRate,
                numberOfChannels,
                numberOfFrames,
                receivedMixedAudio2,
                onMixedAudioFrameAwaiter2);
        });

    // Setup the call
    client1->callAll();
    onAudioFrameAwaiter1.wait(__FILE__, __LINE__);
    onMixedAudioFrameAwaiter1.wait(__FILE__, __LINE__);
    onAudioFrameAwaiter2.wait(__FILE__, __LINE__);
    onMixedAudioFrameAwaiter2.wait(__FILE__, __LINE__);
    client1->hangUpAll();
    client2->hangUpAll();
    client1->setOnMixedAudioFrameReceived([](const void*, int, int, size_t, size_t) {});
    client2->setOnMixedAudioFrameReceived([](const void*, int, int, size_t, size_t) {});

    this_thread::sleep_for(250ms);
    client1->closeSync();
    client2->closeSync();

    // Asserts
    ASSERT_NE(onAddRemoteStreamClient1, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient1->name(), "c2");
    ASSERT_NE(onAddRemoteStreamClient2, nullptr);
    EXPECT_EQ(onAddRemoteStreamClient2->name(), "c1");

    checkReceivedAudio(receivedAudio1, 0);
    checkReceivedAudio(receivedMixedAudio1, 0);
    checkReceivedAudio(receivedAudio2, 0);
    checkReceivedAudio(receivedMixedAudio2, 0);
}

TEST_P(StreamClientTests, audioStream_unidirectional_shouldBeSentAndReceived)
{
    // Initialize the clients
    constexpr int16_t Amplitude = 15000;
    shared_ptr<SinAudioSource> audioSource = make_shared<SinAudioSource>(Amplitude);

    CallbackAwaiter setupAwaiter(2, 15s);
    unique_ptr<StreamClient> client1 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c1", "cd1", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration,
        audioSource);
    unique_ptr<StreamClient> client2 = make_unique<StreamClient>(
        SignalingServerConfiguration::createWithData(m_baseUrl, "c2", "cd2", "chat", "abc"),
        DefaultWebrtcConfiguration,
        m_videoStreamConfiguration);

    client1->setTlsVerificationEnabled(false);
    client2->setTlsVerificationEnabled(false);

    client1->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });
    client2->setOnSignalingConnectionOpened([&] { setupAwaiter.done(); });

    client1->setOnError([](const string& error) { ADD_FAILURE() << error; });
    client2->setOnError([](const string& error) { ADD_FAILURE() << error; });

    client1->connect();
    this_thread::sleep_for(250ms);
    client2->connect();
    setupAwaiter.wait(__FILE__, __LINE__);

    // Setup the callback
    CallbackAwaiter onAudioFrameAwaiter(10, 15s);
    CallbackAwaiter onMixedAudioFrameAwaiter(10, 15s);

    unique_ptr<Client> onAddRemoteStreamClient;

    client1->setOnAddRemoteStream([&](const Client&) { ADD_FAILURE(); });
    client1->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client1->setOnVideoFrameReceived([](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client1->setOnEncodedVideoFrameReceived(
        [](const Client&, const uint8_t*, size_t, VideoCodecType, bool, uint32_t, uint32_t, uint64_t)
        { ADD_FAILURE(); });
    client1->setOnAudioFrameReceived([&](const Client&, const void*, int, int, size_t, size_t) { ADD_FAILURE(); });
    client1->setOnMixedAudioFrameReceived([&](const void*, int, int, size_t, size_t) { ADD_FAILURE(); });

    client2->setOnAddRemoteStream([&](const Client& c) { onAddRemoteStreamClient = make_unique<Client>(c); });
    client2->setOnRemoveRemoteStream([](const Client&) { ADD_FAILURE(); });
    client2->setOnVideoFrameReceived([](const Client&, const cv::Mat&, uint64_t) { ADD_FAILURE(); });
    client2->setOnEncodedVideoFrameReceived(
        [](const Client&, const uint8_t*, size_t, VideoCodecType, bool, uint32_t, uint32_t, uint64_t)
        { ADD_FAILURE(); });
    client2->setOnAudioFrameReceived([&](const Client&, const void*, int, int, size_t, size_t)
                                     { onAudioFrameAwaiter.done(); });
    client2->setOnMixedAudioFrameReceived([&](const void*, int, int, size_t, size_t)
                                          { onMixedAudioFrameAwaiter.done(); });

    // Setup the call
    client1->callAll();
    onAudioFrameAwaiter.wait(__FILE__, __LINE__);
    onMixedAudioFrameAwaiter.wait(__FILE__, __LINE__);
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
        StreamClientTestsParameters{false, false},
        StreamClientTestsParameters{true, false},
        StreamClientTestsParameters{false, true}));
