#include <OpenteraWebrtcNativeClient/Sources/VideoSource.h>

#include <api/video/i420_buffer.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace opentera;
using namespace std;
using namespace cv;

/**
 * @brief Construct a VideoSource
 *
 * @param configuration the configuration applied to the video stream by the image transport layer
 */
VideoSource::VideoSource(VideoSourceConfiguration configuration):
        m_configuration(move(configuration))
{
}

/**
 * @brief Sends a frame to the WebRTC transport layer
 *
 * The frame may or may not be sent depending of the transport layer state
 * Frame will be resized to match the transport layer request
 *
 * @param bgrImg BGR8 encoded frame data
 * @param timestampUs frame timestamp in microseconds
 */
void VideoSource::sendFrame(const Mat& bgrImg, int64_t timestampUs)
{
    cv::Rect roi;
    int outWidth, outHeight;

    // AdaptFrame return true if the transport layer needs a frame
    // Desired resolution is set in out_width and out_height
    if (AdaptFrame(bgrImg.cols, bgrImg.rows, timestampUs, &outWidth, &outHeight,
                   &roi.width, &roi.height, &roi.x, &roi.y)) {

        // I420 only support even resolution so we must make output resolution even!
        outWidth = (outWidth / 2) * 2;
        outHeight = (outHeight / 2) * 2;

        // Convert color space from BGR to YUV
        cv::Mat yuv;
        if (outWidth == roi.width && outHeight == roi.height) {
            cv::cvtColor(bgrImg(roi), yuv, COLOR_BGR2YUV_I420);
        } else {
            cv::Mat m;
            cv::resize(bgrImg(roi), m, cv::Size2i(outWidth, outHeight), 0, 0,
                       outWidth < roi.width ? cv::INTER_AREA : cv::INTER_LINEAR);
            cv::cvtColor(m, yuv, COLOR_BGR2YUV_I420);
        }

        // Convert from OpenCV matrix to webrtc frame
        uint8_t *y = yuv.data;
        uint8_t *u = y + (outWidth * outHeight);
        uint8_t *v = u + (outWidth * outHeight) / 4;
        webrtc::VideoFrame frame(
                webrtc::I420Buffer::Copy(outWidth, outHeight, y, outWidth,
                                         u, outWidth / 2, v, outWidth / 2),
                webrtc::kVideoRotation_0,
                timestampUs
        );

        // Passes the frame to the transport layer
        OnFrame(frame);
    }
}

void VideoSource::AddRef() const
{
}

rtc::RefCountReleaseStatus VideoSource::Release() const
{
    return rtc::RefCountReleaseStatus::kOtherRefsRemained;
}
