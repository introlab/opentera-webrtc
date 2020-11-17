#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_HANDLERS_VIDEO_STREAM_PEER_CONNECTION_HANDLER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_HANDLERS_VIDEO_STREAM_PEER_CONNECTION_HANDLER_H

#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>

namespace introlab
{
    /**
     * @brief A peer connection handler that adds a single video track to the call
     */
    class VideoStreamPeerConnectionHandler : public PeerConnectionHandler
    {
        rtc::scoped_refptr<webrtc::VideoTrackInterface> m_videoTrack;

    public:
        VideoStreamPeerConnectionHandler(const std::string& id,
            const Client& peerClient,
            bool isCaller,
            const std::function<void(const std::string&, sio::message::ptr)>& sendEvent,
            const std::function<void(const std::string&)>& onError,
            const std::function<void(const Client&)>& onClientConnected,
            const std::function<void(const Client&)>& onClientDisconnected,
            rtc::scoped_refptr<webrtc::VideoTrackInterface>  videoTrack);

        ~VideoStreamPeerConnectionHandler() override = default;

        void setPeerConnection(const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection) override;
    };
}

#endif
