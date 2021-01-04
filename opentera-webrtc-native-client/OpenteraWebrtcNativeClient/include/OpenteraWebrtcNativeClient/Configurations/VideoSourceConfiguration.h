#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_VIDEO_SOURCE_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_VIDEO_SOURCE_CONFIGURATION_H

namespace introlab
{
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

        bool needsDenoising() const;
        bool isScreencast() const;

        VideoSourceConfiguration& operator=(const VideoSourceConfiguration& other) = default;
        VideoSourceConfiguration& operator=(VideoSourceConfiguration&& other) = default;
    };

    inline VideoSourceConfiguration VideoSourceConfiguration::create(bool needsDenoising, bool isScreencast)
    {
        return VideoSourceConfiguration(needsDenoising, isScreencast);
    }

    inline bool VideoSourceConfiguration::needsDenoising() const
    {
        return m_needsDenoising;
    }

    inline bool VideoSourceConfiguration::isScreencast() const
    {
        return m_isScreencast;
    }
}

#endif
