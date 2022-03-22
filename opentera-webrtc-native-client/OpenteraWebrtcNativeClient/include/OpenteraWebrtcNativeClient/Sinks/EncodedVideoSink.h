#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_ENCODED_VIDEO_SINK_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_ENCODED_VIDEO_SINK_H

#include <api/video/recordable_encoded_frame.h>
#include <api/video/video_sink_interface.h>

namespace opentera
{
    enum class VideoCodecType
    {
        Generic = 0,
        VP8,
        VP9,
        AV1,
        H264,
        Multiplex,
    };

    using EncodedVideoSinkCallback = std::function<void(
        const uint8_t* data,
        size_t dataSize,
        VideoCodecType codecType,
        bool isKeyFrame,
        uint32_t width,
        uint32_t height,
        uint64_t timestampUs)>;

    /**
     * @brief Class that sinks frame from a webrtc stream
     */
    class EncodedVideoSink : public rtc::VideoSinkInterface<webrtc::RecordableEncodedFrame>
    {
        EncodedVideoSinkCallback m_onFrameReceived;

    public:
        explicit EncodedVideoSink(EncodedVideoSinkCallback onFrameReceived);

        void OnFrame(const webrtc::RecordableEncodedFrame& frame) override;
    };
}

#endif
