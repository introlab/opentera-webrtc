#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_STREAM_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_STREAM_CONFIGURATION_H

#include <api/data_channel_interface.h>

#include <string>
#include <unordered_set>

namespace opentera
{
    /**
     * @brief Represents the available codec for encoding and decoding
     */
    enum class VideoStreamCodec
    {
        VP8,
        VP9,
        H264
    };
    absl::optional<VideoStreamCodec> stringToVideoStreamCodec(std::string_view value);

    /**
     * @brief Represents a video stream configuration.
     */
    class VideoStreamConfiguration
    {
        std::unordered_set<VideoStreamCodec> m_forcedCodecs;  // Empty means all
        bool m_forceGStreamerHardwareAcceleration;
        bool m_useGStreamerSoftwareEncoderDecoder;

        VideoStreamConfiguration(
            std::unordered_set<VideoStreamCodec> forcedCodecs,
            bool forceGStreamerHardwareAcceleration,
            bool useGStreamerSoftwareEncoderDecoder);

    public:
        VideoStreamConfiguration(const VideoStreamConfiguration& other) = default;
        VideoStreamConfiguration(VideoStreamConfiguration&& other) = default;
        virtual ~VideoStreamConfiguration() = default;

        static VideoStreamConfiguration create();
        static VideoStreamConfiguration create(std::unordered_set<VideoStreamCodec> forcedCodecs);
        static VideoStreamConfiguration create(
            std::unordered_set<VideoStreamCodec> forcedCodecs,
            bool forceGStreamerHardwareAcceleration,
            bool useGStreamerSoftwareEncoderDecoder);

        [[nodiscard]] const std::unordered_set<VideoStreamCodec>& forcedCodecs() const;
        [[nodiscard]] bool forceGStreamerHardwareAcceleration() const;
        [[nodiscard]] bool useGStreamerSoftwareEncoderDecoder() const;

        VideoStreamConfiguration& operator=(const VideoStreamConfiguration& other) = default;
        VideoStreamConfiguration& operator=(VideoStreamConfiguration&& other) = default;
    };

    /**
     * @brief Creates a stream configuration with default values.
     * @return A stream configuration with default values
     */
    inline VideoStreamConfiguration VideoStreamConfiguration::create() { return {{}, false, false}; }

    /**
     * @brief Creates a video stream configuration with the specified value.
     *
     * @param forcedCodecs Indicates the codecs that must be used. An empty set means all codecs.
     * @return A video stream configuration with the specified value
     */
    inline VideoStreamConfiguration VideoStreamConfiguration::create(std::unordered_set<VideoStreamCodec> forcedCodecs)
    {
        return {std::move(forcedCodecs), false, false};
    }

    /**
     * @brief Creates a video stream configuration with the specified values.
     *
     * @param forcedCodecs Indicates the codecs that must be used. An empty set means all codecs.
     * @param forceGStreamerHardwareAcceleration Indicates that hardware accelerated codecs must be used. It has no
     * effect when the library is not built with GStreamer.
     * @param useGStreamerSoftwareEncoderDecoder Indicates to use GStreamer software codecs instead of WebRTC ones. It
     * has no effect when the library is not built with GStreamer.
     * @return A video stream channel configuration with the specified values
     */
    inline VideoStreamConfiguration VideoStreamConfiguration::create(
        std::unordered_set<VideoStreamCodec> forcedCodecs,
        bool forceGStreamerHardwareAcceleration,
        bool useGStreamerSoftwareEncoderDecoder)
    {
        return {std::move(forcedCodecs), forceGStreamerHardwareAcceleration, useGStreamerSoftwareEncoderDecoder};
    }

    /**
     * @brief Returns the codecs that must be used. An empty set means all codecs.
     * @return The codecs that must be used
     */
    inline const std::unordered_set<VideoStreamCodec>& VideoStreamConfiguration::forcedCodecs() const
    {
        return m_forcedCodecs;
    }

    /**
     * @brief Indicates that hardware accelerated codecs must be used.
     * @return true if only hardware accelerated codecs can be used.
     */
    inline bool VideoStreamConfiguration::forceGStreamerHardwareAcceleration() const
    {
        return m_forceGStreamerHardwareAcceleration;
    }

    /**
     * @brief Indicates to use GStreamer software codecs instead of WebRTC ones.
     * @return true if GStreamer software codecs must be used instead of WebRTC ones.
     */
    inline bool VideoStreamConfiguration::useGStreamerSoftwareEncoderDecoder() const
    {
        return m_useGStreamerSoftwareEncoderDecoder;
    }

}

#endif
