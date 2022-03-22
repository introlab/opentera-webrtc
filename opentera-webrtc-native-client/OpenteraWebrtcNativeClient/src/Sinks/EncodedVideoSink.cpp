#include <OpenteraWebrtcNativeClient/Sinks/EncodedVideoSink.h>

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
EncodedVideoSink::EncodedVideoSink(EncodedVideoSinkCallback onFrameReceived) : m_onFrameReceived(move(onFrameReceived))
{
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
void EncodedVideoSink::OnFrame(const webrtc::RecordableEncodedFrame& frame)
{
    if (m_onFrameReceived)
    {
        auto encodedBuffer = frame.encoded_buffer();
        auto resolution = frame.resolution();
        m_onFrameReceived(
            encodedBuffer->data(),
            encodedBuffer->size(),
            static_cast<VideoCodecType>(frame.codec()),
            frame.is_key_frame(),
            resolution.width,
            resolution.height,
            frame.render_time().us());
    }
}
