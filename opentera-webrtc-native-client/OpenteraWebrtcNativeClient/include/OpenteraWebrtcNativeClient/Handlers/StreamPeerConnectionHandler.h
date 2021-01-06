#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_HANDLERS_STREAM_PEER_CONNECTION_HANDLER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_HANDLERS_STREAM_PEER_CONNECTION_HANDLER_H

#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>
#include <OpenteraWebrtcNativeClient/Sinks/VideoSink.h>
#include <OpenteraWebrtcNativeClient/Sinks/AudioSink.h>

#include <set>

namespace opentera
{
    class StreamPeerConnectionHandler : public PeerConnectionHandler
    {
        rtc::scoped_refptr<webrtc::VideoTrackInterface> m_videoTrack;
        rtc::scoped_refptr<webrtc::AudioTrackInterface> m_audioTrack;

        std::function<void(const Client&)> m_onAddRemoteStream;
        std::function<void(const Client&)> m_onRemoveRemoteStream;

        std::unique_ptr<VideoSink> m_videoSink;
        std::unique_ptr<AudioSink> m_audioSink;

        std::set<rtc::scoped_refptr<webrtc::MediaStreamInterface>> m_streams;

    public:
        StreamPeerConnectionHandler(
                std::string id,
                Client peerClient,
                bool isCaller,
                std::function<void(const std::string&, const sio::message::ptr&)> sendEvent,
                std::function<void(const std::string&)> onError,
                std::function<void(const Client&)> onClientConnected,
                std::function<void(const Client&)> onClientDisconnected,
                rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack,
                rtc::scoped_refptr<webrtc::AudioTrackInterface> audioTrack,
                std::function<void(const Client&)> onAddRemoteStream,
                std::function<void(const Client&)> onRemoveRemoteStream,
                const std::function<void(const Client&, const cv::Mat&, uint64_t)>& onVideoFrameReceived,
                const std::function<void(const Client&, const void*, int, int, size_t, size_t)>& onAudioFrameReceived);

        ~StreamPeerConnectionHandler() override;

        void setPeerConnection(const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection) override;

        // Observer methods
        void OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override;
        void OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override;
    };
}

#endif
