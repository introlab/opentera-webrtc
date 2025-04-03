#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_SINK_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_SINK_H

#include <api/video/video_frame.h>
#include <api/video/video_sink_interface.h>
#include <api/video/video_source_interface.h>
#include <opencv2/core.hpp>

namespace opentera
{

    using VideoSinkCallback = std::function<void(const cv::Mat&, uint64_t)>;

    /**
     * @brief Class that sinks frame from a webrtc stream
     */
    class VideoSink : public rtc::VideoSinkInterface<webrtc::VideoFrame>
    {
        VideoSinkCallback m_onFrameReceived;
        rtc::VideoSinkWants m_wants;
        cv::Mat m_bgrImg;
        cv::Mat m_bgrRotatedImg;

    public:
        explicit VideoSink(VideoSinkCallback onFrameReceived);

        void OnFrame(const webrtc::VideoFrame& frame) override;
        [[nodiscard]] rtc::VideoSinkWants wants() const;
    };

    /**
     * @brief get frame requirements for this sink
     * @return frame requirements for this sink
     */
    inline rtc::VideoSinkWants VideoSink::wants() const { return m_wants; }

}

#endif
