#include <OpenteraWebrtcNativeClient/Configurations/AudioSourceConfiguration.h>

#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(AudioSourceConfigurationTests, create_shouldSetNullOpt)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create();

    EXPECT_EQ(testee.echoCancellation(), absl::nullopt);
    EXPECT_EQ(testee.autoGainControl(), absl::nullopt);
    EXPECT_EQ(testee.noiseSuppression(), absl::nullopt);
    EXPECT_EQ(testee.highpassFilter(), absl::nullopt);
    EXPECT_EQ(testee.stereoSwapping(), absl::nullopt);
    EXPECT_EQ(testee.typingDetection(), absl::nullopt);
    EXPECT_EQ(testee.residualEchoDetector(), absl::nullopt);
}

TEST(AudioSourceConfigurationTests, create_echoCancellationAutoGainControl_shouldSetTheAttributes)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(true, false, absl::nullopt, absl::nullopt,
            absl::nullopt, absl::nullopt, absl::nullopt);

    EXPECT_EQ(testee.echoCancellation(), true);
    EXPECT_EQ(testee.autoGainControl(), false);
    EXPECT_EQ(testee.noiseSuppression(), absl::nullopt);
    EXPECT_EQ(testee.highpassFilter(), absl::nullopt);
    EXPECT_EQ(testee.stereoSwapping(), absl::nullopt);
    EXPECT_EQ(testee.typingDetection(), absl::nullopt);
    EXPECT_EQ(testee.residualEchoDetector(), absl::nullopt);

    auto options = static_cast<cricket::AudioOptions>(testee);
    EXPECT_EQ(options.echo_cancellation, true);
    EXPECT_EQ(options.auto_gain_control, false);
    EXPECT_EQ(options.noise_suppression, absl::nullopt);
    EXPECT_EQ(options.highpass_filter, absl::nullopt);
    EXPECT_EQ(options.stereo_swapping, absl::nullopt);
    EXPECT_EQ(options.typing_detection, absl::nullopt);
    EXPECT_EQ(options.residual_echo_detector, absl::nullopt);
}

TEST(AudioSourceConfigurationTests, create_noiseSuppressionHighpassFilter_shouldSetTheAttributes)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(absl::nullopt, absl::nullopt, true, false,
            absl::nullopt, absl::nullopt, absl::nullopt);

    EXPECT_EQ(testee.echoCancellation(), absl::nullopt);
    EXPECT_EQ(testee.autoGainControl(), absl::nullopt);
    EXPECT_EQ(testee.noiseSuppression(), true);
    EXPECT_EQ(testee.highpassFilter(), false);
    EXPECT_EQ(testee.stereoSwapping(), absl::nullopt);
    EXPECT_EQ(testee.typingDetection(), absl::nullopt);
    EXPECT_EQ(testee.residualEchoDetector(), absl::nullopt);

    auto options = static_cast<cricket::AudioOptions>(testee);
    EXPECT_EQ(options.echo_cancellation, absl::nullopt);
    EXPECT_EQ(options.auto_gain_control, absl::nullopt);
    EXPECT_EQ(options.noise_suppression, true);
    EXPECT_EQ(options.highpass_filter, false);
    EXPECT_EQ(options.stereo_swapping, absl::nullopt);
    EXPECT_EQ(options.typing_detection, absl::nullopt);
    EXPECT_EQ(options.residual_echo_detector, absl::nullopt);
}

TEST(AudioSourceConfigurationTests, create_stereoSwappingTypingDetection_shouldSetTheAttributes)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(absl::nullopt, absl::nullopt, absl::nullopt,
            absl::nullopt, true, false, absl::nullopt);

    EXPECT_EQ(testee.echoCancellation(), absl::nullopt);
    EXPECT_EQ(testee.autoGainControl(), absl::nullopt);
    EXPECT_EQ(testee.noiseSuppression(), absl::nullopt);
    EXPECT_EQ(testee.highpassFilter(), absl::nullopt);
    EXPECT_EQ(testee.stereoSwapping(), true);
    EXPECT_EQ(testee.typingDetection(), false);
    EXPECT_EQ(testee.residualEchoDetector(), absl::nullopt);

    auto options = static_cast<cricket::AudioOptions>(testee);
    EXPECT_EQ(options.echo_cancellation, absl::nullopt);
    EXPECT_EQ(options.auto_gain_control, absl::nullopt);
    EXPECT_EQ(options.noise_suppression, absl::nullopt);
    EXPECT_EQ(options.highpass_filter, absl::nullopt);
    EXPECT_EQ(options.stereo_swapping, true);
    EXPECT_EQ(options.typing_detection, false);
    EXPECT_EQ(options.residual_echo_detector, absl::nullopt);
}

TEST(AudioSourceConfigurationTests, create_residualEchoDetector_shouldSetTheAttributes)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(absl::nullopt, absl::nullopt, absl::nullopt,
            absl::nullopt, absl::nullopt, absl::nullopt, true);

    EXPECT_EQ(testee.echoCancellation(), absl::nullopt);
    EXPECT_EQ(testee.autoGainControl(), absl::nullopt);
    EXPECT_EQ(testee.noiseSuppression(), absl::nullopt);
    EXPECT_EQ(testee.highpassFilter(), absl::nullopt);
    EXPECT_EQ(testee.stereoSwapping(), absl::nullopt);
    EXPECT_EQ(testee.typingDetection(), absl::nullopt);
    EXPECT_EQ(testee.residualEchoDetector(), true);

    auto options = static_cast<cricket::AudioOptions>(testee);
    EXPECT_EQ(options.echo_cancellation, absl::nullopt);
    EXPECT_EQ(options.auto_gain_control, absl::nullopt);
    EXPECT_EQ(options.noise_suppression, absl::nullopt);
    EXPECT_EQ(options.highpass_filter, absl::nullopt);
    EXPECT_EQ(options.stereo_swapping, absl::nullopt);
    EXPECT_EQ(options.typing_detection, absl::nullopt);
    EXPECT_EQ(options.residual_echo_detector, true);
}


