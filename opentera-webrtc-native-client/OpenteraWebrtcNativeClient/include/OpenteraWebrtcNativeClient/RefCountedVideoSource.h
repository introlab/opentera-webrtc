#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_REF_COUNTED_VIDEO_SOURCE_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_REF_COUNTED_VIDEO_SOURCE_H

#include <media/base/adapted_video_track_source.h>
#include <rtc_base/ref_counted_object.h>

namespace rtc {
    typedef RefCountedObject<rtc::AdaptedVideoTrackSource> RefCountedVideoSource;
}

#endif //OPENTERA_WEBRTC_NATIVE_CLIENT_REF_COUNTED_VIDEO_SOURCE_H
