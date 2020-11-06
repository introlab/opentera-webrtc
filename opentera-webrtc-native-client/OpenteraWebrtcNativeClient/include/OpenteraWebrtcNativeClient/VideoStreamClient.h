#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_STREAM_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_STREAM_CLIENT_H

#include <OpenteraWebrtcNativeClient/SignallingClient.h>

namespace introlab
{
    class VideoStreamClient: public SignallingClient
    {
    public:
        VideoStreamClient(const SignallingServerConfiguration& signallingServerConfiguration,
                          const WebrtcConfiguration& webrtcConfiguration);
        ~VideoStreamClient() override = default;

        DECLARE_NOT_COPYABLE(VideoStreamClient);
        DECLARE_NOT_MOVABLE(VideoStreamClient);

    protected:
        std::unique_ptr<PeerConnectionHandler> createPeerConnectionHandler(const std::string& id,
                const Client& peerClient, bool isCaller) override;
    };
}

#endif // OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_STREAM_CLIENT_H
