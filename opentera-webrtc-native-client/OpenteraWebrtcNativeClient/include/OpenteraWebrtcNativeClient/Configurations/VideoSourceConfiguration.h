#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_VIDEO_SOURCE_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_VIDEO_SOURCE_CONFIGURATION_H

namespace opentera
{
    /**
     * @brief Represents a configuration of a video source that can be added to a WebRTC call.
     */
    class VideoSourceConfiguration
    {
        bool m_needsDenoising;
        bool m_isScreencast;

        VideoSourceConfiguration(bool needsDenoising, bool isScreencast);

    public:
        VideoSourceConfiguration(const VideoSourceConfiguration& other) = default;
        VideoSourceConfiguration(VideoSourceConfiguration&& other) = default;
        virtual ~VideoSourceConfiguration() = default;

        static VideoSourceConfiguration create(bool needsDenoising, bool isScreencast);

        [[nodiscard]] bool needsDenoising() const;
        [[nodiscard]] bool isScreencast() const;

        VideoSourceConfiguration& operator=(const VideoSourceConfiguration& other) = default;
        VideoSourceConfiguration& operator=(VideoSourceConfiguration&& other) = default;
    };

    /**
     * @brief Creates a video source configuration with the specified values.
     *
     * @param needsDenoising Indicates if this source needs denoising
     * @param isScreencast Indicates if this source is screencast
     * @return A video source configuration with the specified values
     */
    inline VideoSourceConfiguration VideoSourceConfiguration::create(bool needsDenoising, bool isScreencast)
    {
        return {needsDenoising, isScreencast};
    }

    /**
     * @brief Indicates if this source needs denoising.
     * @return true if this source needs denoising
     */
    inline bool VideoSourceConfiguration::needsDenoising() const { return m_needsDenoising; }

    /**
     * @brief Indicates if this source is screencast.
     * @return true if this source is a screencast
     */
    inline bool VideoSourceConfiguration::isScreencast() const { return m_isScreencast; }
}

#endif
