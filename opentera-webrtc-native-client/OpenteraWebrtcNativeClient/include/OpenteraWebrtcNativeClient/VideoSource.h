#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_SOURCE_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_SOURCE_H

#include <media/base/adapted_video_track_source.h>
#include <rtc_base/ref_counted_object.h>
#include <opencv2/core/mat.hpp>

namespace introlab {

    /**
     * @brief Represent a video source that can be added to a WebRTC call
     *
     * Pass a ref_ptr to an instance of this to the StreamClient and call sendFrame for each of your frame
     */
    class VideoSource : public rtc::AdaptedVideoTrackSource
    {
        bool m_needsDenoising;
        bool m_isScreenCast;

    public:
        VideoSource(bool needsDenoising, bool isScreencast);
        void sendFrame(const cv::Mat& bgrImg, int64_t timestampUs);

        bool is_screencast() const override;
        absl::optional<bool> needs_denoising() const override;
        bool remote() const override;
        webrtc::MediaSourceInterface::SourceState state() const override;

        // Methods to fake a ref counted object, so the Python binding is easier to make because we can use a shared_ptr
        void AddRef() const override;
        rtc::RefCountReleaseStatus Release() const override;
    };

    /**
     * @brief indicates if this source is screencast
     * @return true if this source is a screencast
     */
    inline bool VideoSource::is_screencast() const{ return m_isScreenCast; }

    /**
     * @brief indicates if this source needs denoising
     * @return true if this source needs denoising
     */
    inline absl::optional<bool> VideoSource::needs_denoising() const { return m_needsDenoising; }

    /**
     * @brief indicates if this source is remote
     * @return always false, the source is local
     */
    inline bool VideoSource::remote() const { return false; }

    /**
     * @brief indicates if this source is live
     * @return always kLive, the source is live
     */
    inline webrtc::MediaSourceInterface::SourceState VideoSource::state() const {
        return webrtc::MediaSourceInterface::kLive; }
}

#endif
