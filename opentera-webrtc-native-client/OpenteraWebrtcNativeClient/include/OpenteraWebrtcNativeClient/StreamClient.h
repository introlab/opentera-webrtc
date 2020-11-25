#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_STREAM_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_STREAM_CLIENT_H

#include <OpenteraWebrtcNativeClient/VideoSource.h>
#include <OpenteraWebrtcNativeClient/SignallingClient.h>

#include <memory>

namespace introlab
{
    /**
     * @brief a signaling client to join a webrtc room and stream a video source
     */
    class StreamClient: public SignallingClient
    {
        std::shared_ptr<VideoSource> m_videoSource;

    public:
        StreamClient(const SignallingServerConfiguration& signallingServerConfiguration,
                     const WebrtcConfiguration& webrtcConfiguration,
                     const std::shared_ptr<VideoSource>& videoSource);
        ~StreamClient() override = default;

        DECLARE_NOT_COPYABLE(StreamClient);
        DECLARE_NOT_MOVABLE(StreamClient);

    protected:
        std::unique_ptr<PeerConnectionHandler> createPeerConnectionHandler(const std::string& id,
                const Client& peerClient, bool isCaller) override;
    };
}

#endif
