#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_ROS_VIDEO_SOURCE_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_ROS_VIDEO_SOURCE_H

#include <OpenteraWebrtcNativeClient/RefCountedVideoSource.h>
#include <sensor_msgs/Image.h>

namespace introlab
{
    class RosVideoSource : public rtc::RefCountedVideoSource
    {
    public:
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        bool is_screencast() const override;
        bool remote() const override;
        absl::optional<bool> needs_denoising() const override;
        webrtc::MediaSourceInterface::SourceState state() const override;
    };
}

#endif // OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_ROS_VIDEO_SOURCE_H
