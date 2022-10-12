#include <OpenteraWebrtcNativeClient/Configurations/AudioSourceConfiguration.h>

#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(AudioSourceConfigurationTests, create_shouldSetNullOpt)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(10);

    EXPECT_EQ(testee.soundCardTotalDelayMs(), 10);
    EXPECT_EQ(testee.echoCancellation(), absl::nullopt);
    EXPECT_EQ(testee.autoGainControl(), absl::nullopt);
    EXPECT_EQ(testee.noiseSuppression(), absl::nullopt);
    EXPECT_EQ(testee.highpassFilter(), absl::nullopt);
    EXPECT_EQ(testee.stereoSwapping(), absl::nullopt);
}

TEST(AudioSourceConfigurationTests, create_echoCancellation_shouldSetTheAttributes)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(
        10,
        true,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt);

    EXPECT_EQ(testee.soundCardTotalDelayMs(), 10);
    EXPECT_EQ(testee.echoCancellation(), true);
    EXPECT_EQ(testee.autoGainControl(), absl::nullopt);
    EXPECT_EQ(testee.noiseSuppression(), absl::nullopt);
    EXPECT_EQ(testee.highpassFilter(), absl::nullopt);
    EXPECT_EQ(testee.stereoSwapping(), absl::nullopt);
    EXPECT_EQ(testee.transientSuppression(), absl::nullopt);

    auto options = static_cast<cricket::AudioOptions>(testee);
    EXPECT_EQ(options.echo_cancellation, true);
    EXPECT_EQ(options.auto_gain_control, absl::nullopt);
    EXPECT_EQ(options.noise_suppression, absl::nullopt);
    EXPECT_EQ(options.highpass_filter, absl::nullopt);
    EXPECT_EQ(options.stereo_swapping, absl::nullopt);

    auto config = static_cast<webrtc::AudioProcessing::Config>(testee);
    EXPECT_EQ(config.echo_canceller.enabled, true);
    EXPECT_EQ(config.gain_controller2.enabled, false);
    EXPECT_EQ(config.noise_suppression.enabled, false);
    EXPECT_EQ(config.high_pass_filter.enabled, false);
    EXPECT_EQ(config.transient_suppression.enabled, false);
}

TEST(AudioSourceConfigurationTests, create_autoGainControl_shouldSetTheAttributes)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(
        10,
        absl::nullopt,
        true,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt);

    EXPECT_EQ(testee.soundCardTotalDelayMs(), 10);
    EXPECT_EQ(testee.echoCancellation(), absl::nullopt);
    EXPECT_EQ(testee.autoGainControl(), true);
    EXPECT_EQ(testee.noiseSuppression(), absl::nullopt);
    EXPECT_EQ(testee.highpassFilter(), absl::nullopt);
    EXPECT_EQ(testee.stereoSwapping(), absl::nullopt);
    EXPECT_EQ(testee.transientSuppression(), absl::nullopt);

    auto options = static_cast<cricket::AudioOptions>(testee);
    EXPECT_EQ(options.echo_cancellation, absl::nullopt);
    EXPECT_EQ(options.auto_gain_control, true);
    EXPECT_EQ(options.noise_suppression, absl::nullopt);
    EXPECT_EQ(options.highpass_filter, absl::nullopt);
    EXPECT_EQ(options.stereo_swapping, absl::nullopt);

    auto config = static_cast<webrtc::AudioProcessing::Config>(testee);
    EXPECT_EQ(config.echo_canceller.enabled, false);
    EXPECT_EQ(config.gain_controller2.enabled, true);
    EXPECT_EQ(config.noise_suppression.enabled, false);
    EXPECT_EQ(config.high_pass_filter.enabled, false);
    EXPECT_EQ(config.transient_suppression.enabled, false);
}

TEST(AudioSourceConfigurationTests, create_noiseSuppression_shouldSetTheAttributes)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(
        10,
        absl::nullopt,
        absl::nullopt,
        true,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt);

    EXPECT_EQ(testee.soundCardTotalDelayMs(), 10);
    EXPECT_EQ(testee.echoCancellation(), absl::nullopt);
    EXPECT_EQ(testee.autoGainControl(), absl::nullopt);
    EXPECT_EQ(testee.noiseSuppression(), true);
    EXPECT_EQ(testee.highpassFilter(), absl::nullopt);
    EXPECT_EQ(testee.stereoSwapping(), absl::nullopt);
    EXPECT_EQ(testee.transientSuppression(), absl::nullopt);

    auto options = static_cast<cricket::AudioOptions>(testee);
    EXPECT_EQ(options.echo_cancellation, absl::nullopt);
    EXPECT_EQ(options.auto_gain_control, absl::nullopt);
    EXPECT_EQ(options.noise_suppression, true);
    EXPECT_EQ(options.highpass_filter, absl::nullopt);
    EXPECT_EQ(options.stereo_swapping, absl::nullopt);

    auto config = static_cast<webrtc::AudioProcessing::Config>(testee);
    EXPECT_EQ(config.echo_canceller.enabled, false);
    EXPECT_EQ(config.gain_controller2.enabled, false);
    EXPECT_EQ(config.noise_suppression.enabled, true);
    EXPECT_EQ(config.high_pass_filter.enabled, false);
    EXPECT_EQ(config.transient_suppression.enabled, false);
}

TEST(AudioSourceConfigurationTests, create_highpassFilter_shouldSetTheAttributes)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(
        10,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt,
        true,
        absl::nullopt,
        absl::nullopt);

    EXPECT_EQ(testee.soundCardTotalDelayMs(), 10);
    EXPECT_EQ(testee.echoCancellation(), absl::nullopt);
    EXPECT_EQ(testee.autoGainControl(), absl::nullopt);
    EXPECT_EQ(testee.noiseSuppression(), absl::nullopt);
    EXPECT_EQ(testee.highpassFilter(), true);
    EXPECT_EQ(testee.stereoSwapping(), absl::nullopt);
    EXPECT_EQ(testee.transientSuppression(), absl::nullopt);

    auto options = static_cast<cricket::AudioOptions>(testee);
    EXPECT_EQ(options.echo_cancellation, absl::nullopt);
    EXPECT_EQ(options.auto_gain_control, absl::nullopt);
    EXPECT_EQ(options.noise_suppression, absl::nullopt);
    EXPECT_EQ(options.highpass_filter, true);
    EXPECT_EQ(options.stereo_swapping, absl::nullopt);

    auto config = static_cast<webrtc::AudioProcessing::Config>(testee);
    EXPECT_EQ(config.echo_canceller.enabled, false);
    EXPECT_EQ(config.gain_controller2.enabled, false);
    EXPECT_EQ(config.noise_suppression.enabled, false);
    EXPECT_EQ(config.high_pass_filter.enabled, true);
    EXPECT_EQ(config.transient_suppression.enabled, false);
}

TEST(AudioSourceConfigurationTests, create_stereoSwapping_shouldSetTheAttributes)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(
        10,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt,
        true,
        absl::nullopt);

    EXPECT_EQ(testee.soundCardTotalDelayMs(), 10);
    EXPECT_EQ(testee.echoCancellation(), absl::nullopt);
    EXPECT_EQ(testee.autoGainControl(), absl::nullopt);
    EXPECT_EQ(testee.noiseSuppression(), absl::nullopt);
    EXPECT_EQ(testee.highpassFilter(), absl::nullopt);
    EXPECT_EQ(testee.stereoSwapping(), true);
    EXPECT_EQ(testee.transientSuppression(), absl::nullopt);

    auto options = static_cast<cricket::AudioOptions>(testee);
    EXPECT_EQ(options.echo_cancellation, absl::nullopt);
    EXPECT_EQ(options.auto_gain_control, absl::nullopt);
    EXPECT_EQ(options.noise_suppression, absl::nullopt);
    EXPECT_EQ(options.highpass_filter, absl::nullopt);
    EXPECT_EQ(options.stereo_swapping, true);

    auto config = static_cast<webrtc::AudioProcessing::Config>(testee);
    EXPECT_EQ(config.echo_canceller.enabled, false);
    EXPECT_EQ(config.gain_controller2.enabled, false);
    EXPECT_EQ(config.noise_suppression.enabled, false);
    EXPECT_EQ(config.high_pass_filter.enabled, false);
    EXPECT_EQ(config.transient_suppression.enabled, false);
}

TEST(AudioSourceConfigurationTests, create_transientSuppression_shouldSetTheAttributes)
{
    AudioSourceConfiguration testee = AudioSourceConfiguration::create(
        10,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt,
        absl::nullopt,
        true);

    EXPECT_EQ(testee.soundCardTotalDelayMs(), 10);
    EXPECT_EQ(testee.echoCancellation(), absl::nullopt);
    EXPECT_EQ(testee.autoGainControl(), absl::nullopt);
    EXPECT_EQ(testee.noiseSuppression(), absl::nullopt);
    EXPECT_EQ(testee.highpassFilter(), absl::nullopt);
    EXPECT_EQ(testee.stereoSwapping(), absl::nullopt);
    EXPECT_EQ(testee.transientSuppression(), true);

    auto options = static_cast<cricket::AudioOptions>(testee);
    EXPECT_EQ(options.echo_cancellation, absl::nullopt);
    EXPECT_EQ(options.auto_gain_control, absl::nullopt);
    EXPECT_EQ(options.noise_suppression, absl::nullopt);
    EXPECT_EQ(options.highpass_filter, absl::nullopt);
    EXPECT_EQ(options.stereo_swapping, absl::nullopt);

    auto config = static_cast<webrtc::AudioProcessing::Config>(testee);
    EXPECT_EQ(config.echo_canceller.enabled, false);
    EXPECT_EQ(config.gain_controller2.enabled, false);
    EXPECT_EQ(config.noise_suppression.enabled, false);
    EXPECT_EQ(config.high_pass_filter.enabled, false);
    EXPECT_EQ(config.transient_suppression.enabled, true);
}
