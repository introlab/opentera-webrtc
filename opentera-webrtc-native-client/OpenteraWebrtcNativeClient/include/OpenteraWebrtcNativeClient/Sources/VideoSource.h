#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_VIDEO_SOURCE_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_VIDEO_SOURCE_H

#include <OpenteraWebrtcNativeClient/Configurations/VideoSourceConfiguration.h>
#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>

#include <media/base/adapted_video_track_source.h>
#include <opencv2/core/mat.hpp>
#include <rtc_base/ref_counted_object.h>

namespace opentera
{

    /**
     * @brief Represents a video source that can be added to a WebRTC call.
     *
     * Pass a shared_ptr to an instance of this to the StreamClient and call
     * sendFrame for each of your frame.
     */
    class VideoSource : public rtc::AdaptedVideoTrackSource
    {
        VideoSourceConfiguration m_configuration;
        cv::Mat m_yuvImg;
        cv::Mat m_resizedImg;

    public:
        explicit VideoSource(VideoSourceConfiguration configuration);
        ~VideoSource() override = default;

        DECLARE_NOT_COPYABLE(VideoSource);
        DECLARE_NOT_MOVABLE(VideoSource);

        void sendFrame(const cv::Mat& bgrImg, int64_t timestampUs);

        bool is_screencast() const override;
        absl::optional<bool> needs_denoising() const override;
        bool remote() const override;
        webrtc::MediaSourceInterface::SourceState state() const override;

        // Methods to fake a ref counted object, so the Python binding is easier to
        // make because we can use a shared_ptr
        void AddRef() const override;
        rtc::RefCountReleaseStatus Release() const override;
    };

    /**
     * @brief Indicates if this source is screencast.
     * @return true if this source is a screencast
     */
    inline bool VideoSource::is_screencast() const { return m_configuration.isScreencast(); }

    /**
     * @brief Indicates if this source needs denoising.
     * @return true if this source needs denoising
     */
    inline absl::optional<bool> VideoSource::needs_denoising() const { return m_configuration.needsDenoising(); }

    /**
     * @brief Indicates if this source is remote.
     * @return Always false, the source is local
     */
    inline bool VideoSource::remote() const { return false; }

    /**
     * @brief Indicates if this source is live.
     * @return Always kLive, the source is live
     */
    inline webrtc::MediaSourceInterface::SourceState VideoSource::state() const
    {
        return webrtc::MediaSourceInterface::kLive;
    }
}

#endif
