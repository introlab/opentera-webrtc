#include <OpenteraWebrtcNativeClient/Sources/AudioSource.h>
#include <OpenteraWebrtcNativeClient/Sinks/AudioSink.h>

#include <gtest/gtest.h>

#include <vector>

using namespace opentera;
using namespace std;

TEST(AudioSourceTests, constructor_shouldOnlySupportValidBitsPerSample)
{
    EXPECT_NO_THROW(AudioSource(AudioSourceConfiguration::create(0), 8, 48000, 1));
    EXPECT_NO_THROW(AudioSource(AudioSourceConfiguration::create(0), 16, 48000, 1));
    EXPECT_NO_THROW(AudioSource(AudioSourceConfiguration::create(0), 32, 48000, 1));
    EXPECT_THROW(AudioSource(AudioSourceConfiguration::create(0), 7, 48000, 1), runtime_error);
}

TEST(AudioSourceTests, configuration_shouldTheSpecifiedValues)
{
    auto configuration = AudioSourceConfiguration::create(0, true, true, true, true, true, true, true, true);
    AudioSource testee(configuration, 16, 48000, 1);
    EXPECT_EQ(testee.configuration().soundCardTotalDelayMs(), 0);
    EXPECT_EQ(testee.configuration().echoCancellation(), true);
    EXPECT_EQ(testee.configuration().autoGainControl(), true);
    EXPECT_EQ(testee.configuration().noiseSuppression(), true);
    EXPECT_EQ(testee.configuration().highpassFilter(), true);
    EXPECT_EQ(testee.configuration().stereoSwapping(), true);
    EXPECT_EQ(testee.configuration().typingDetection(), true);
    EXPECT_EQ(testee.configuration().residualEchoDetector(), true);
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
    auto configuration = AudioSourceConfiguration::create(0, true, true, true, true, true, true, true, true);
    AudioSource testee(configuration, 16, 48000, 1);
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
