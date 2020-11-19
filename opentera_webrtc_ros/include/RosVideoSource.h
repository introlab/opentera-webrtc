#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_ROS_VIDEO_SOURCE_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_ROS_VIDEO_SOURCE_H

#include <OpenteraWebrtcNativeClient/RefCountedVideoSource.h>
#include <sensor_msgs/Image.h>

namespace introlab
{
    /**
     * @brief A webrtc video source that sinks images from a ROS topic
     *
     * Usage: pass an rtc_scoped_ptr to an instance of this to the VideoStreamClient constructor.
     * Use the imageCallback as a ROS topic subscriber callback.
     */
    class RosVideoSource : public rtc::RefCountedVideoSource
    {
        // Store denoising and screencast params
        bool m_needsDenoising;
        bool m_isScreenCast;

    public:
        RosVideoSource(bool needsDenoising, bool isScreenCast);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        /**
         * @brief indicates if this source is screencast
         * @return true if this source is a screencast
         */
        inline bool is_screencast() const override { return m_isScreenCast; };

        /**
         * @brief indicates if this source needs denoising
         * @return true if this source needs denoising
         */
        inline absl::optional<bool> needs_denoising() const override { return m_needsDenoising; };

        /**
         * @brief indicates if this source is remote
         * @return always false, the source is local
         */
        inline bool remote() const override { return false; };

        /**
         * @brief indicates if this source is live
         * @return always kLive, the source is live
         */
        inline webrtc::MediaSourceInterface::SourceState state() const override {
            return webrtc::MediaSourceInterface::kLive; };
    };
}

#endif
