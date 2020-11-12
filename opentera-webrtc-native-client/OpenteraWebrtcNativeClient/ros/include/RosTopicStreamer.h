#pragma once

#include <RosVideoSource.h>
#include <OpenteraWebrtcNativeClient/VideoStreamClient.h>

namespace introlab {

    class RosTopicStreamer
    {
        std::unique_ptr<VideoStreamClient> m_signallingClient;
        rtc::scoped_refptr<RosVideoSource> m_videoSource;

    public:
        RosTopicStreamer();
        virtual ~RosTopicStreamer();
    };

}