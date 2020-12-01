#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_HANDLERS_VIDEO_STREAM_PEER_CONNECTION_HANDLER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_HANDLERS_VIDEO_STREAM_PEER_CONNECTION_HANDLER_H

#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>

namespace introlab
{
    class VideoStreamPeerConnectionHandler : public PeerConnectionHandler
    {
        rtc::scoped_refptr<webrtc::VideoTrackInterface> m_videoTrack;

    public:
        VideoStreamPeerConnectionHandler(std::string id,
            Client peerClient,
            bool isCaller,
            std::function<void(const std::string&, const sio::message::ptr&)> sendEvent,
            std::function<void(const std::string&)> onError,
            std::function<void(const Client&)> onClientConnected,
            std::function<void(const Client&)> onClientDisconnected,
            rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack);

        ~VideoStreamPeerConnectionHandler() override = default;

        void setPeerConnection(const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection) override;
    };
}

#endif
