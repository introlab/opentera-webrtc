#include <OpenteraWebrtcNativeClient/Sinks/VideoSink.h>

#include <utility>
#include <opencv2/imgproc.hpp>

using namespace introlab;
using namespace webrtc;
using namespace rtc;
using namespace std;

/**
 * @brief Construct a VideoSink
 *
 * @param onFrameReceived callback function that gets called whenever a frame is received
 */
VideoSink::VideoSink(function<void(const cv::Mat&, uint64_t)>  onFrameReceived) :
        m_onFrameReceived(move(onFrameReceived))
{
    m_wants.rotation_applied = true;

    // Specify we want resolution to be multiple of 2 for I420
    m_wants.resolution_alignment = 2;
}

/**
 * @brief Process incoming frames from webrtc
 *
 * This function is called by the webrtc transport layer whenever a frame is available.
 * It convert YUV frame data from the I420 buffer to BGR data and call the callback function with a cv::Mat
 *
 * @param frame available webrtc frame
 */
void VideoSink::OnFrame(const VideoFrame& frame)
{
    // Transform data from 3 array in I420 buffer to one cv::Mat in yuv
    int width = frame.width();
    int height = frame.height();

    cv::Mat yuvImg;
    yuvImg.create(height * 3 / 2, width, CV_8UC1);

    uint8_t* yData = const_cast<uint8_t *>(frame.video_frame_buffer()->GetI420()->DataY());
    uint8_t* uData = const_cast<uint8_t *>(frame.video_frame_buffer()->GetI420()->DataU());
    uint8_t* vData = const_cast<uint8_t *>(frame.video_frame_buffer()->GetI420()->DataV());

    size_t yDataLength = width * height;
    size_t uvDataLength = yDataLength / 4;

    uint8_t* yDataStart = yuvImg.data;
    uint8_t* uDataStart = yDataStart + yDataLength;
    uint8_t* vDataStart = uDataStart + uvDataLength;

    memcpy(yDataStart, yData, yDataLength);
    memcpy(uDataStart, uData, uvDataLength);
    memcpy(vDataStart, vData, uvDataLength);

    // Convert yuv color space to bgr
    cv::Mat bgrImg;
    cv::cvtColor(yuvImg, bgrImg, cv::COLOR_YUV2BGR_I420);

    // Pass bgr frame to image callback
    m_onFrameReceived(bgrImg, frame.timestamp_us());
}
