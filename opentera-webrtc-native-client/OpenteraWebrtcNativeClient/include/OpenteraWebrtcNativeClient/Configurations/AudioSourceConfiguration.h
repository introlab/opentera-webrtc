#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_AUDIO_SOURCE_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_AUDIO_SOURCE_CONFIGURATION_H

#include <api/audio_options.h>

namespace introlab
{
    class AudioSourceConfiguration
    {
        absl::optional<bool> m_echoCancellation;
        absl::optional<bool> m_autoGainControl;
        absl::optional<bool> m_noiseSuppression;
        absl::optional<bool> m_highpassFilter;
        absl::optional<bool> m_stereoSwapping;
        absl::optional<bool> m_typingDetection;
        absl::optional<bool> m_residualEchoDetector;

        AudioSourceConfiguration(absl::optional<bool> echoCancellation,
                 absl::optional<bool> autoGainControl,
                 absl::optional<bool> noiseSuppression,
                 absl::optional<bool> highpassFilter,
                 absl::optional<bool> stereoSwapping,
                 absl::optional<bool> typingDetection,
                 absl::optional<bool> residualEchoDetector);

    public:
        AudioSourceConfiguration(const AudioSourceConfiguration& other) = default;
        AudioSourceConfiguration(AudioSourceConfiguration&& other) = default;
        virtual ~AudioSourceConfiguration() = default;

        static AudioSourceConfiguration create();
        static AudioSourceConfiguration create(absl::optional<bool> echoCancellation,
                absl::optional<bool> autoGainControl,
                absl::optional<bool> noiseSuppression,
                absl::optional<bool> highpassFilter,
                absl::optional<bool> stereoSwapping,
                absl::optional<bool> typingDetection,
                absl::optional<bool> residualEchoDetector);

        absl::optional<bool> echoCancellation() const;
        absl::optional<bool> autoGainControl() const;
        absl::optional<bool> noiseSuppression() const;
        absl::optional<bool> highpassFilter() const;
        absl::optional<bool> stereoSwapping() const;
        absl::optional<bool> typingDetection() const;
        absl::optional<bool> residualEchoDetector() const;

        explicit operator cricket::AudioOptions() const;

        AudioSourceConfiguration& operator=(const AudioSourceConfiguration& other) = default;
        AudioSourceConfiguration& operator=(AudioSourceConfiguration&& other) = default;
    };

    inline AudioSourceConfiguration AudioSourceConfiguration::create()
    {
        return AudioSourceConfiguration(absl::nullopt, absl::nullopt, absl::nullopt, absl::nullopt, absl::nullopt,
                absl::nullopt, absl::nullopt);
    }

    inline AudioSourceConfiguration AudioSourceConfiguration::create(absl::optional<bool> echoCancellation,
                                           absl::optional<bool> autoGainControl,
                                           absl::optional<bool> noiseSuppression,
                                           absl::optional<bool> highpassFilter,
                                           absl::optional<bool> stereoSwapping,
                                           absl::optional<bool> typingDetection,
                                           absl::optional<bool> residualEchoDetector)
    {
        return AudioSourceConfiguration(echoCancellation, autoGainControl, noiseSuppression, highpassFilter,
                stereoSwapping, typingDetection, residualEchoDetector);
    }

    inline absl::optional<bool> AudioSourceConfiguration::echoCancellation() const
    {
        return m_echoCancellation;
    }

    inline absl::optional<bool> AudioSourceConfiguration::autoGainControl() const
    {
        return m_autoGainControl;
    }

    inline absl::optional<bool> AudioSourceConfiguration::noiseSuppression() const
    {
        return m_noiseSuppression;
    }

    inline absl::optional<bool> AudioSourceConfiguration::highpassFilter() const
    {
        return m_highpassFilter;
    }

    inline absl::optional<bool> AudioSourceConfiguration::stereoSwapping() const
    {
        return m_stereoSwapping;
    }

    inline absl::optional<bool> AudioSourceConfiguration::typingDetection() const
    {
        return m_typingDetection;
    }

    inline absl::optional<bool> AudioSourceConfiguration::residualEchoDetector() const
    {
        return m_residualEchoDetector;
    }
}

#endif
