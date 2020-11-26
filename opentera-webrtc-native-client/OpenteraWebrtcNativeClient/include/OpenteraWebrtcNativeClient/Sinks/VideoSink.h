#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_SINK_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_SINK_H

#include <api/video/video_sink_interface.h>
#include <api/video/video_frame.h>
#include <api/video/video_source_interface.h>
#include <opencv2/core.hpp>

namespace introlab {

    /**
     * @brief Class that sinks frame from a webrtc stream
     *
     * Build this by passing a callback that process received frames and pass the instance to a stream client.
     */
    class VideoSink : public rtc::VideoSinkInterface<webrtc::VideoFrame>
    {
        std::function<void(const cv::Mat&, uint64_t)> m_onFrameReceived;
        rtc::VideoSinkWants m_wants;

    public:
        explicit VideoSink(std::function<void(const cv::Mat&, uint64_t)>  onFrameReceived);

        void OnFrame(const webrtc::VideoFrame& frame) override;
        rtc::VideoSinkWants wants() const;
    };

    /**
     * @brief get frame requirements for this sink
     * @return frame requirements for this sink
     */
    inline rtc::VideoSinkWants VideoSink::wants() const
    {
        return m_wants;
    }

}


#endif
