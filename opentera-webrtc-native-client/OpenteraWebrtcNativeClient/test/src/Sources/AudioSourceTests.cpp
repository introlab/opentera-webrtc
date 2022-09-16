#include <OpenteraWebrtcNativeClient/Sinks/AudioSink.h>
#include <OpenteraWebrtcNativeClient/Sources/AudioSource.h>

#include <gtest/gtest.h>

#include <rtc_base/ref_counted_object.h>

#include <vector>

using namespace opentera;
using namespace std;

class AudioTransportMock : public webrtc::AudioTransport
{
public:
    vector<vector<int8_t>> m_capturedData;
    vector<size_t> m_bytesPerSample;
    vector<size_t> m_sampleRate;
    vector<size_t> m_numberOfChannels;
    vector<size_t> m_numberOfFrames;
    vector<uint32_t> m_totalDelayMS;
    vector<bool> m_keyPressed;

    int32_t RecordedDataIsAvailable(
        const void* audioSamples,
        const size_t nSamples,
        const size_t nBytesPerSample,
        const size_t nChannels,
        const uint32_t samplesPerSec,
        const uint32_t totalDelayMS,
        const int32_t clockDrift,
        const uint32_t currentMicLevel,
        const bool keyPressed,
        uint32_t& newMicLevel) override
    {
        m_capturedData.emplace_back(
            reinterpret_cast<const int8_t*>(audioSamples),
            reinterpret_cast<const int8_t*>(audioSamples) + nChannels * nSamples);

        m_bytesPerSample.emplace_back(nBytesPerSample);
        m_sampleRate.emplace_back(samplesPerSec);
        m_numberOfChannels.emplace_back(nChannels);
        m_numberOfFrames.emplace_back(nSamples);
        m_totalDelayMS.emplace_back(totalDelayMS);
        m_keyPressed.emplace_back(keyPressed);

        return 0;
    }

    virtual int32_t NeedMorePlayData(
        const size_t nSamples,
        const size_t nBytesPerSample,
        const size_t nChannels,
        const uint32_t samplesPerSec,
        void* audioSamples,
        size_t& nSamplesOut,
        int64_t* elapsed_time_ms,
        int64_t* ntp_time_ms) override
    {
        return -1;
    }

    virtual void PullRenderData(
        int bits_per_sample,
        int sample_rate,
        size_t number_of_channels,
        size_t number_of_frames,
        void* audio_data,
        int64_t* elapsed_time_ms,
        int64_t* ntp_time_ms) override
    {
    }
};

TEST(AudioSourceTests, constructor_shouldOnlySupportValidBitsPerSample)
{
    EXPECT_NO_THROW(AudioSource(AudioSourceConfiguration::create(0), 8, 48000, 1));
    EXPECT_NO_THROW(AudioSource(AudioSourceConfiguration::create(0), 16, 48000, 1));
    EXPECT_NO_THROW(AudioSource(AudioSourceConfiguration::create(0), 32, 48000, 1));
    EXPECT_THROW(AudioSource(AudioSourceConfiguration::create(0), 7, 48000, 1), runtime_error);
}

TEST(AudioSourceTests, configuration_shouldTheSpecifiedValues)
{
    auto configuration = AudioSourceConfiguration::create(0, true, true, true, true, true, true);
    AudioSource testee(configuration, 16, 48000, 1);
    EXPECT_EQ(testee.configuration().soundCardTotalDelayMs(), 0);
    EXPECT_EQ(testee.configuration().echoCancellation(), true);
    EXPECT_EQ(testee.configuration().autoGainControl(), true);
    EXPECT_EQ(testee.configuration().noiseSuppression(), true);
    EXPECT_EQ(testee.configuration().highpassFilter(), true);
    EXPECT_EQ(testee.configuration().stereoSwapping(), true);
    EXPECT_EQ(testee.configuration().transientSuppression(), true);
}

TEST(AudioSourceTests, remote_shouldReturnFalse)
{
    AudioSource testee(AudioSourceConfiguration::create(0), 16, 48000, 1);
    EXPECT_EQ(testee.remote(), false);
}

TEST(AudioSourceTests, state_shouldReturnLive)
{
    AudioSource testee(AudioSourceConfiguration::create(0), 16, 48000, 1);
    EXPECT_EQ(testee.state(), webrtc::MediaSourceInterface::SourceState::kLive);
}

TEST(AudioSourceTests, options_shouldTheSpecifiedValues)
{
    auto configuration = AudioSourceConfiguration::create(0, true, true, true, true, true, true);
    AudioSource testee(configuration, 16, 48000, 1);
    auto options = testee.options();
    EXPECT_EQ(options.echo_cancellation, true);
    EXPECT_EQ(options.auto_gain_control, true);
    EXPECT_EQ(options.noise_suppression, true);
    EXPECT_EQ(options.highpass_filter, true);
    EXPECT_EQ(options.stereo_swapping, true);
}

TEST(AudioSourceTests, bytesPerSample_shouldTheSpecifiedValue)
{
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(0), 8, 48000, 1).bytesPerSample(), 1);
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(0), 16, 48000, 1).bytesPerSample(), 2);
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(0), 32, 48000, 1).bytesPerSample(), 4);
}

TEST(AudioSourceTests, bytesPerFrame_shouldTheSpecifiedValue)
{
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(0), 8, 48000, 2).bytesPerFrame(), 2);
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(0), 16, 48000, 2).bytesPerFrame(), 4);
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(0), 32, 48000, 2).bytesPerFrame(), 8);
}

TEST(AudioSourceTests, sendFrame_shouldCreate10msFrame)
{
    AudioSource testee(AudioSourceConfiguration::create(10), 8, 400, 2);
    rtc::scoped_refptr<OpenteraAudioDeviceModule> adm(new rtc::RefCountedObject<OpenteraAudioDeviceModule>);
    AudioTransportMock audioTransportMock;
    adm->RegisterAudioCallback(&audioTransportMock);
    testee.setAudioDeviceModule(adm);

    int8_t data1[] = {1, 2, 3, 4, 5, 6};
    int8_t data2[] = {7, 8, 9, 10, 11, 12};
    int8_t data3[] = {13, 14, 15, 16, 17, 18};

    testee.sendFrame(data1, 3, true);
    testee.sendFrame(data2, 3, false);
    testee.sendFrame(data3, 3, true);

    ASSERT_EQ(audioTransportMock.m_capturedData.size(), 2);
    EXPECT_EQ(audioTransportMock.m_capturedData[0], vector<int8_t>({1, 2, 3, 4, 5, 6, 7, 8}));
    EXPECT_EQ(audioTransportMock.m_capturedData[1], vector<int8_t>({9, 10, 11, 12, 13, 14, 15, 16}));

    EXPECT_EQ(audioTransportMock.m_bytesPerSample, vector<size_t>({1, 1}));
    EXPECT_EQ(audioTransportMock.m_sampleRate, vector<size_t>({400, 400}));
    EXPECT_EQ(audioTransportMock.m_numberOfChannels, vector<size_t>({2, 2}));
    EXPECT_EQ(audioTransportMock.m_numberOfFrames, vector<size_t>({4, 4}));
    EXPECT_EQ(audioTransportMock.m_totalDelayMS, vector<uint32_t>({10, 10}));
    EXPECT_EQ(audioTransportMock.m_keyPressed, vector<bool>({false, true}));
}
