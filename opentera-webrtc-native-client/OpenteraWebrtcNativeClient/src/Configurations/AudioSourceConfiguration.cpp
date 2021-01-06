#include <OpenteraWebrtcNativeClient/Configurations/AudioSourceConfiguration.h>

using namespace opentera;
using namespace std;

AudioSourceConfiguration::AudioSourceConfiguration(absl::optional<bool> echoCancellation,
        absl::optional<bool> autoGainControl,
        absl::optional<bool> noiseSuppression,
        absl::optional<bool> highpassFilter,
        absl::optional<bool> stereoSwapping,
        absl::optional<bool> typingDetection,
        absl::optional<bool> residualEchoDetector) :
        m_echoCancellation(echoCancellation),
        m_autoGainControl(autoGainControl),
        m_noiseSuppression(noiseSuppression),
        m_highpassFilter(highpassFilter),
        m_stereoSwapping(stereoSwapping),
        m_typingDetection(typingDetection),
        m_residualEchoDetector(residualEchoDetector)
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
    options.typing_detection = m_typingDetection;
    options.residual_echo_detector = m_residualEchoDetector;

    return options;
}
