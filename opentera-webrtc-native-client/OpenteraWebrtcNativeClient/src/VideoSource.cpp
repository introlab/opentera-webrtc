#include <OpenteraWebrtcNativeClient/VideoSource.h>
#include <api/video/i420_buffer.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace introlab;
using namespace cv;

/**
 * @brief Construct a VideoSource
 *
 * @param needsDenoising denoising should be applied to the video stream by the image transport layer
 * @param isScreenCast the transport layer should be configured to stream a screen rather then a camera
 */
VideoSource::VideoSource(bool needsDenoising, bool isScreenCast):
        m_needsDenoising(needsDenoising), m_isScreenCast(isScreenCast)
{

}

/**
 * @brief Sends a frame to the WebRTC transport layer
 *
 * The frame may or may not be sent depending of the transport layer state
 * Frame will be resized to match the transport layer request
 *
 * @param bgr_img BGR8 encoded frame data
 * @param timestamp_us frame timestamp in microseconds
 */
void VideoSource::sendFrame(const Mat& bgr_img, int64_t timestamp_us)
{
    cv::Rect roi;
    int out_width, out_height;

    // AdaptFrame return true if the transport layer needs a frame
    // Desired resolution is set in out_width and out_height
    if (AdaptFrame(bgr_img.cols, bgr_img.rows, timestamp_us, &out_width, &out_height,
                   &roi.width, &roi.height, &roi.x, &roi.y)) {

        // I420 only support even resolution so we must make output resolution even!
        out_width = (out_width / 2) * 2;
        out_height = (out_height / 2) * 2;

        // Convert color space from BGR to YUV
        cv::Mat yuv;
        if (out_width == roi.width && out_height == roi.height) {
            cv::cvtColor(bgr_img(roi), yuv, COLOR_BGR2YUV_I420);
        } else {
            cv::Mat m;
            cv::resize(bgr_img(roi), m, cv::Size2i(out_width, out_height), 0, 0,
                       out_width < roi.width ? cv::INTER_AREA : cv::INTER_LINEAR);
            cv::cvtColor(m, yuv, COLOR_BGR2YUV_I420);
        }

        // Convert from OpenCV matrix to webrtc frame
        uint8_t *y = yuv.data;
        uint8_t *u = y + (out_width * out_height);
        uint8_t *v = u + (out_width * out_height) / 4;
        webrtc::VideoFrame frame(
                webrtc::I420Buffer::Copy(out_width, out_height, y, out_width,
                                         u, out_width / 2, v, out_width / 2),
                webrtc::kVideoRotation_0,
                timestamp_us
        );

        // Passes the frame to the transport layer
        OnFrame(frame);
    }
}