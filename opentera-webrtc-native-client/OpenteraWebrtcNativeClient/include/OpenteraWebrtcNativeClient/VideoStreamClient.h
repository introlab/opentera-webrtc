#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_STREAM_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_STREAM_CLIENT_H

#include <OpenteraWebrtcNativeClient/VideoSource.h>
#include <OpenteraWebrtcNativeClient/SignallingClient.h>

#include <memory>

namespace introlab
{
    /**
     * @brief a signaling client to join a webrtc room and stream a video source
     */
    class VideoStreamClient: public SignallingClient
    {
        std::shared_ptr<VideoSource> m_videoSource;

    public:
        VideoStreamClient(SignallingServerConfiguration signallingServerConfiguration,
                          WebrtcConfiguration webrtcConfiguration,
                          std::shared_ptr<VideoSource> videoSource);
        ~VideoStreamClient() override = default;

        DECLARE_NOT_COPYABLE(VideoStreamClient);
        DECLARE_NOT_MOVABLE(VideoStreamClient);

    protected:
        std::unique_ptr<PeerConnectionHandler> createPeerConnectionHandler(const std::string& id,
                const Client& peerClient, bool isCaller) override;
    };
}

#endif
