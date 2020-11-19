#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_STREAM_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_STREAM_CLIENT_H

#include <media/base/adapted_video_track_source.h>
#include <OpenteraWebrtcNativeClient/SignallingClient.h>

namespace introlab
{
    /**
     * @brief a signaling client to join a webrtc room and stream a video source
     */
    class VideoStreamClient: public SignallingClient
    {
        rtc::scoped_refptr<rtc::RefCountedObject<rtc::AdaptedVideoTrackSource>> m_videoSource;

    public:
        VideoStreamClient(const SignallingServerConfiguration& signallingServerConfiguration,
                          const WebrtcConfiguration& webrtcConfiguration,
                          rtc::scoped_refptr<rtc::RefCountedObject<rtc::AdaptedVideoTrackSource>> videoSource);
        ~VideoStreamClient() override = default;

        DECLARE_NOT_COPYABLE(VideoStreamClient);
        DECLARE_NOT_MOVABLE(VideoStreamClient);

    protected:
        std::unique_ptr<PeerConnectionHandler> createPeerConnectionHandler(const std::string& id,
                const Client& peerClient, bool isCaller) override;
    };
}

#endif
