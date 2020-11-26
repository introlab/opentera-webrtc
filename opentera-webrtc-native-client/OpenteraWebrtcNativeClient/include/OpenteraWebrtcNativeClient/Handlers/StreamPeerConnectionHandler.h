#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_HANDLERS_STREAM_PEER_CONNECTION_HANDLER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_HANDLERS_STREAM_PEER_CONNECTION_HANDLER_H

#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>
#include <OpenteraWebrtcNativeClient/Sinks/VideoSink.h>

namespace introlab
{
    class StreamPeerConnectionHandler : public PeerConnectionHandler
    {
        rtc::scoped_refptr<webrtc::VideoTrackInterface> m_videoTrack;
        std::shared_ptr<VideoSink> m_videoSink;

        rtc::scoped_refptr<webrtc::AudioTrackInterface> m_audioTrack;
        std::shared_ptr<webrtc::AudioTrackSinkInterface> m_audioSink;

    public:
        StreamPeerConnectionHandler(
                const std::string& id,
                const Client& peerClient,
                bool isCaller,
                const std::function<void(const std::string&, sio::message::ptr)>& sendEvent,
                const std::function<void(const std::string&)>& onError,
                const std::function<void(const Client&)>& onClientConnected,
                const std::function<void(const Client&)>& onClientDisconnected,
                rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack = nullptr,
                std::shared_ptr<VideoSink> videoSink = nullptr,
                rtc::scoped_refptr<webrtc::AudioTrackInterface> audioTrack = nullptr,
                std::shared_ptr<webrtc::AudioTrackSinkInterface> audioSink = nullptr);

        ~StreamPeerConnectionHandler() override = default;

        void setPeerConnection(const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection) override;

        // Observer methods
        void OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override;
        void OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override;
    };
}

#endif
