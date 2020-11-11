#include <OpenteraWebrtcNativeClient/Sources/VideoSource.h>

using namespace introlab;

void VideoSource::putFrame(webrtc::VideoFrame const &frame)
{
    OnFrame(frame);
}

bool VideoSource::is_screencast() const
{
    return false;
}

bool VideoSource::remote() const
{
    return false;
}

absl::optional<bool> VideoSource::needs_denoising() const
{
    return false;
}