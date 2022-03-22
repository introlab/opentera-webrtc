#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_AUDIO_SOURCE_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_AUDIO_SOURCE_CONFIGURATION_H

#include <api/audio_options.h>
#include <modules/audio_processing/include/audio_processing.h>

namespace opentera
{
    /**
     * @brief Represents a configuration of an audio source that can be added to a WebRTC call.
     */
    class AudioSourceConfiguration
    {
        uint32_t m_soundCardTotalDelayMs;
        absl::optional<bool> m_echoCancellation;
        absl::optional<bool> m_autoGainControl;
        absl::optional<bool> m_noiseSuppression;
        absl::optional<bool> m_highpassFilter;
        absl::optional<bool> m_stereoSwapping;
        absl::optional<bool> m_typingDetection;
        absl::optional<bool> m_residualEchoDetector;
        absl::optional<bool> m_transientSuppression;

        AudioSourceConfiguration(
            uint32_t soundCardTotalDelayMs,
            absl::optional<bool> echoCancellation,
            absl::optional<bool> autoGainControl,
            absl::optional<bool> noiseSuppression,
            absl::optional<bool> highpassFilter,
            absl::optional<bool> stereoSwapping,
            absl::optional<bool> typingDetection,
            absl::optional<bool> residualEchoDetector,
            absl::optional<bool> transientSuppression);

    public:
        AudioSourceConfiguration(const AudioSourceConfiguration& other) = default;
        AudioSourceConfiguration(AudioSourceConfiguration&& other) = default;
        virtual ~AudioSourceConfiguration() = default;

        static AudioSourceConfiguration create(uint32_t soundCardTotalDelayMs);
        static AudioSourceConfiguration create(
            uint32_t soundCardTotalDelayMs,
            absl::optional<bool> echoCancellation,
            absl::optional<bool> autoGainControl,
            absl::optional<bool> noiseSuppression,
            absl::optional<bool> highpassFilter,
            absl::optional<bool> stereoSwapping,
            absl::optional<bool> typingDetection,
            absl::optional<bool> residualEchoDetector,
            absl::optional<bool> transientSuppression);

        uint32_t soundCardTotalDelayMs() const;
        absl::optional<bool> echoCancellation() const;
        absl::optional<bool> autoGainControl() const;
        absl::optional<bool> noiseSuppression() const;
        absl::optional<bool> highpassFilter() const;
        absl::optional<bool> stereoSwapping() const;
        absl::optional<bool> typingDetection() const;
        absl::optional<bool> residualEchoDetector() const;
        absl::optional<bool> transientSuppression() const;

        explicit operator cricket::AudioOptions() const;
        explicit operator webrtc::AudioProcessing::Config() const;

        AudioSourceConfiguration& operator=(const AudioSourceConfiguration& other) = default;
        AudioSourceConfiguration& operator=(AudioSourceConfiguration&& other) = default;
    };

    /**
     * @brief Creates an audio source configuration with default values.
     * @param soundCardTotalDelayMs The sum of the playback and recording delays.
     * @return An audio source configuration with default values
     */
    inline AudioSourceConfiguration AudioSourceConfiguration::create(uint32_t soundCardTotalDelayMs)
    {
        return AudioSourceConfiguration(
            soundCardTotalDelayMs,
            absl::nullopt,
            absl::nullopt,
            absl::nullopt,
            absl::nullopt,
            absl::nullopt,
            absl::nullopt,
            absl::nullopt,
            absl::nullopt);
    }

    /**
     * @brief Creates an audio source configuration with the specified values.
     *
     * @param soundCardTotalDelayMs The sum of the playback and recording delays.
     * @param echoCancellation Enable or disable the echo cancellation
     * @param autoGainControl Enable or disable the auto gain control
     * @param noiseSuppression Enable or disable the noise suppression
     * @param highpassFilter Enable or disable the highpass filter
     * @param stereoSwapping Enable or disable the stereo swapping
     * @param typingDetection Enable or disable the typing detection
     * @param residualEchoDetector Enable or disable the residual echo detector
     * @param transientSuppression Enable or disable the transient suppression
     * @return An audio source configuration with the specified values
     */
    inline AudioSourceConfiguration AudioSourceConfiguration::create(
        uint32_t soundCardTotalDelayMs,
        absl::optional<bool> echoCancellation,
        absl::optional<bool> autoGainControl,
        absl::optional<bool> noiseSuppression,
        absl::optional<bool> highpassFilter,
        absl::optional<bool> stereoSwapping,
        absl::optional<bool> typingDetection,
        absl::optional<bool> residualEchoDetector,
        absl::optional<bool> transientSuppression)
    {
        return AudioSourceConfiguration(
            soundCardTotalDelayMs,
            echoCancellation,
            autoGainControl,
            noiseSuppression,
            highpassFilter,
            stereoSwapping,
            typingDetection,
            residualEchoDetector,
            transientSuppression);
    }

    /**
     * @brief Returns the sum of the playback and recording delays.
     * @return The sum of the playback and recording delays
     */
    inline uint32_t AudioSourceConfiguration::soundCardTotalDelayMs() const { return m_soundCardTotalDelayMs; }

    /**
     * @brief Indicates if the echo cancellation is enabled.
     * @return true if the echo cancellation is enabled
     */
    inline absl::optional<bool> AudioSourceConfiguration::echoCancellation() const { return m_echoCancellation; }

    /**
     * @brief Indicates if the auto gain control is enabled.
     * @return true if the auto gain control is enabled
     */
    inline absl::optional<bool> AudioSourceConfiguration::autoGainControl() const { return m_autoGainControl; }

    /**
     * @brief Indicates if the noise suppression is enabled.
     * @return true if the noise suppression is enabled
     */
    inline absl::optional<bool> AudioSourceConfiguration::noiseSuppression() const { return m_noiseSuppression; }

    /**
     * @brief Indicates if the highpass filter is enabled.
     * @return true if the highpass filter is enabled
     */
    inline absl::optional<bool> AudioSourceConfiguration::highpassFilter() const { return m_highpassFilter; }

    /**
     * @brief Indicates if the stereo swapping is enabled.
     * @return true if the stereo swapping is enabled
     */
    inline absl::optional<bool> AudioSourceConfiguration::stereoSwapping() const { return m_stereoSwapping; }

    /**
     * @brief Indicates if the typing detection is enabled.
     * @return true if the typing detection is enabled
     */
    inline absl::optional<bool> AudioSourceConfiguration::typingDetection() const { return m_typingDetection; }

    /**
     * @brief Indicates if the residual echo detector is enabled.
     * @return true if the residual echo detector is enabled
     */
    inline absl::optional<bool> AudioSourceConfiguration::residualEchoDetector() const
    {
        return m_residualEchoDetector;
    }

    /**
     * @brief Indicates if the transient suppression is enabled.
     * @return true if the transient suppression is enabled
     */
    inline absl::optional<bool> AudioSourceConfiguration::transientSuppression() const
    {
        return m_transientSuppression;
    }
}

#endif
