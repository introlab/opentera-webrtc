#include <OpenteraWebrtcNativeClient/Configurations/AudioSourceConfiguration.h>

using namespace opentera;
using namespace std;

AudioSourceConfiguration::AudioSourceConfiguration(
    uint32_t soundCardTotalDelayMs,
    absl::optional<bool> echoCancellation,
    absl::optional<bool> autoGainControl,
    absl::optional<bool> noiseSuppression,
    absl::optional<bool> highpassFilter,
    absl::optional<bool> stereoSwapping,
    absl::optional<bool> transientSuppression)
    : m_soundCardTotalDelayMs(soundCardTotalDelayMs),
      m_echoCancellation(echoCancellation),
      m_autoGainControl(autoGainControl),
      m_noiseSuppression(noiseSuppression),
      m_highpassFilter(highpassFilter),
      m_stereoSwapping(stereoSwapping),
      m_transientSuppression(transientSuppression)
{
}

/**
 * Converts a AudioSourceConfiguration to a cricket::AudioOptions.
 * @return The converted cricket::AudioOptions
 */
AudioSourceConfiguration::operator cricket::AudioOptions() const
{
    cricket::AudioOptions options;
    options.echo_cancellation = m_echoCancellation;
    options.auto_gain_control = m_autoGainControl;
    options.noise_suppression = m_noiseSuppression;
    options.highpass_filter = m_highpassFilter;
    options.stereo_swapping = m_stereoSwapping;

    return options;
}

/**
 * Converts a AudioSourceConfiguration to a webrtc::AudioProcessing::Config.
 * @return The converted webrtc::AudioProcessing::Config
 */
AudioSourceConfiguration::operator webrtc::AudioProcessing::Config() const
{
    webrtc::AudioProcessing::Config config;
    if (m_echoCancellation.has_value())
    {
        config.echo_canceller.enabled = m_echoCancellation.value();
    }
    if (m_autoGainControl.has_value())
    {
        config.gain_controller2.enabled = m_autoGainControl.value();
    }
    if (m_noiseSuppression.has_value())
    {
        config.noise_suppression.enabled = m_noiseSuppression.value();
    }
    if (m_highpassFilter.has_value())
    {
        config.high_pass_filter.enabled = m_highpassFilter.value();
    }
    if (m_transientSuppression.has_value())
    {
        config.transient_suppression.enabled = m_transientSuppression.value();
    }

    return config;
}
