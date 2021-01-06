#include <OpenteraWebrtcNativeClient/Sources/AudioSource.h>
#include <OpenteraWebrtcNativeClient/Sinks/AudioSink.h>

#include <gtest/gtest.h>

#include <vector>

using namespace opentera;
using namespace std;

TEST(AudioSourceTests, constructor_shouldOnlySupportValidBitsPerSample)
{
    EXPECT_NO_THROW(AudioSource(AudioSourceConfiguration::create(), 8, 48000, 1));
    EXPECT_NO_THROW(AudioSource(AudioSourceConfiguration::create(), 16, 48000, 1));
    EXPECT_NO_THROW(AudioSource(AudioSourceConfiguration::create(), 32, 48000, 1));
    EXPECT_THROW(AudioSource(AudioSourceConfiguration::create(), 7, 48000, 1), runtime_error);
}

TEST(AudioSourceTests, remote_shouldReturnFalse)
{
    AudioSource testee(AudioSourceConfiguration::create(), 16, 48000, 1);
    EXPECT_EQ(testee.remote(), false);
}

TEST(AudioSourceTests, state_shouldReturnLive)
{
    AudioSource testee(AudioSourceConfiguration::create(), 16, 48000, 1);
    EXPECT_EQ(testee.state(), webrtc::MediaSourceInterface::SourceState::kLive);
}

TEST(AudioSourceTests, options_shouldTheSpecifiedValues)
{
    AudioSource testee(AudioSourceConfiguration::create(true, true, true, true, true, true, true), 16, 48000, 1);
    auto options = testee.options();
    EXPECT_EQ(options.echo_cancellation, true);
    EXPECT_EQ(options.auto_gain_control, true);
    EXPECT_EQ(options.noise_suppression, true);
    EXPECT_EQ(options.highpass_filter, true);
    EXPECT_EQ(options.stereo_swapping, true);
    EXPECT_EQ(options.typing_detection, true);
    EXPECT_EQ(options.residual_echo_detector, true);
}

TEST(AudioSourceTests, bytesPerSample_shouldTheSpecifiedValue)
{
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(), 8, 48000, 1).bytesPerSample(), 1);
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(), 16, 48000, 1).bytesPerSample(), 2);
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(), 32, 48000, 1).bytesPerSample(), 4);
}

TEST(AudioSourceTests, bytesPerFrame_shouldTheSpecifiedValue)
{
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(), 8, 48000, 2).bytesPerFrame(), 2);
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(), 16, 48000, 2).bytesPerFrame(), 4);
    EXPECT_EQ(AudioSource(AudioSourceConfiguration::create(), 32, 48000, 2).bytesPerFrame(), 8);
}

TEST(AudioSourceTests, sendFrame_shouldCreate10msFrame)
{
    AudioSource testee(AudioSourceConfiguration::create(), 8, 400, 2);
    vector<int8_t> sinkData1;
    vector<int8_t> sinkData2;

    AudioSink sink1([&](const void* audioData, int bitsPerSample, int sampleRate, size_t numberOfChannels,
            size_t numberOfFrames)
    {
        EXPECT_EQ(bitsPerSample, 8);
        EXPECT_EQ(sampleRate, 400);
        EXPECT_EQ(numberOfChannels, 2);
        EXPECT_EQ(numberOfFrames, 4);

        sinkData1.insert(sinkData1.end(), reinterpret_cast<const int8_t*>(audioData),
                reinterpret_cast<const int8_t*>(audioData) + 8);
    });

    AudioSink sink2([&](const void* audioData, int bitsPerSample, int sampleRate, size_t numberOfChannels,
                       size_t numberOfFrames)
    {
        EXPECT_EQ(bitsPerSample, 8);
        EXPECT_EQ(sampleRate, 400);
        EXPECT_EQ(numberOfChannels, 2);
        EXPECT_EQ(numberOfFrames, 4);

        sinkData2.insert(sinkData2.end(), reinterpret_cast<const int8_t*>(audioData),
                         reinterpret_cast<const int8_t*>(audioData) + 8);
    });

    int8_t data1[] = { 1, 2, 3, 4, 5, 6 };
    int8_t data2[] = { 7, 8, 9, 10, 11, 12 };
    int8_t data3[] = { 13, 14, 15, 16, 17, 18 };

    testee.AddSink(&sink1);
    testee.AddSink(&sink2);

    testee.sendFrame(data1, 3);
    testee.sendFrame(data2, 3);

    testee.RemoveSink(&sink2);
    testee.sendFrame(data3, 3);

    EXPECT_EQ(sinkData1, vector<int8_t>({ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 }));
    EXPECT_EQ(sinkData2, vector<int8_t>({ 1, 2, 3, 4, 5, 6, 7, 8 }));
}


