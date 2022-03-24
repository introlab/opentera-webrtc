#include <OpenteraWebrtcNativeClient/Sinks/VideoSink.h>

#include <libyuv.h>
#include <opencv2/imgproc.hpp>
#include <utility>

using namespace opentera;
using namespace rtc;
using namespace std;

/**
 * @brief Construct a VideoSink
 *
 * @param onFrameReceived callback function that gets called whenever a frame is
 * received
 */
VideoSink::VideoSink(VideoSinkCallback onFrameReceived) : m_onFrameReceived(move(onFrameReceived))
{
    m_wants.rotation_applied = true;
}

/**
 * @brief Process incoming frames from webrtc
 *
 * This function is called by the webrtc transport layer whenever a frame is
 * available. It convert YUV frame data from the I420 buffer to BGR data and
 * call the callback function with a cv::Mat
 *
 * @param frame available webrtc frame
 */
void VideoSink::OnFrame(const webrtc::VideoFrame& frame)
{
    // Transform data from 3 array in I420 buffer to one cv::Mat in yuv
    m_bgrImg.create(frame.height(), frame.width(), CV_8UC3);

    int err = libyuv::ConvertFromI420(
        frame.video_frame_buffer()->GetI420()->DataY(),
        frame.video_frame_buffer()->GetI420()->StrideY(),
        frame.video_frame_buffer()->GetI420()->DataU(),
        frame.video_frame_buffer()->GetI420()->StrideU(),
        frame.video_frame_buffer()->GetI420()->DataV(),
        frame.video_frame_buffer()->GetI420()->StrideV(),
        m_bgrImg.data,
        m_bgrImg.step[0],
        frame.width(),
        frame.height(),
        libyuv::FOURCC_24BG);

    if (err == 0 && m_onFrameReceived)
    {
        // Pass bgr frame to image callback if conversion worked
        m_onFrameReceived(m_bgrImg, frame.timestamp_us());
    }
}
