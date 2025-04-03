#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_HANDLERS_STREAM_PEER_CONNECTION_HANDLER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_HANDLERS_STREAM_PEER_CONNECTION_HANDLER_H

#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>
#include <OpenteraWebrtcNativeClient/Sinks/VideoSink.h>
#include <OpenteraWebrtcNativeClient/Sinks/EncodedVideoSink.h>
#include <OpenteraWebrtcNativeClient/Sinks/AudioSink.h>

#include <set>

namespace opentera
{
    using VideoFrameReceivedCallback = std::function<void(const Client&, const cv::Mat&, uint64_t)>;
    using EncodedVideoFrameReceivedCallback = std::function<void(
        const Client& client,
        const uint8_t* data,
        size_t dataSize,
        VideoCodecType codecType,
        bool isKeyFrame,
        uint32_t width,
        uint32_t height,
        uint64_t timestampUs)>;
    using AudioFrameReceivedCallback = std::function<void(
        const Client& client,
        const void* audioData,
        int bitsPerSample,
        int sampleRate,
        size_t numberOfChannels,
        size_t numberOfFrames)>;

    class StreamPeerConnectionHandler : public PeerConnectionHandler
    {
        bool m_offerToReceiveVideo;
        bool m_offerToReceiveAudio;

        rtc::scoped_refptr<webrtc::VideoTrackInterface> m_videoTrack;
        rtc::scoped_refptr<webrtc::AudioTrackInterface> m_audioTrack;

        std::function<void(const Client&)> m_onAddRemoteStream;
        std::function<void(const Client&)> m_onRemoveRemoteStream;

        std::unique_ptr<VideoSink> m_videoSink;
        std::unique_ptr<EncodedVideoSink> m_encodedVideoSink;
        std::unique_ptr<AudioSink> m_audioSink;

        std::set<rtc::scoped_refptr<webrtc::MediaStreamTrackInterface>> m_tracks;

    public:
        StreamPeerConnectionHandler(
            std::string id,
            Client peerClient,
            bool isCaller,
            bool hasOnMixedAudioFrameReceivedCallback,
            SignalingClient& m_signalingClient,
            std::function<void(const std::string&)> onError,
            std::function<void(const Client&)> onClientConnected,
            std::function<void(const Client&)> onClientDisconnected,
            std::function<void(const Client&)> onClientConnectionFailed,
            rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack,
            rtc::scoped_refptr<webrtc::AudioTrackInterface> audioTrack,
            std::function<void(const Client&)> onAddRemoteStream,
            std::function<void(const Client&)> onRemoveRemoteStream,
            const VideoFrameReceivedCallback& onVideoFrameReceived,
            const EncodedVideoFrameReceivedCallback& onEncodedVideoFrameReceived,
            const AudioFrameReceivedCallback& onAudioFrameReceived);

        ~StreamPeerConnectionHandler() override;

        void setPeerConnection(const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection) override;

        void setAllLocalAudioTracksEnabled(bool enabled);
        void setAllRemoteAudioTracksEnabled(bool enabled);
        void setAllVideoTracksEnabled(bool enabled);

        // Observer methods
        void OnTrack(rtc::scoped_refptr<webrtc::RtpTransceiverInterface> transceiver) override;
        void OnRemoveTrack(rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver) override;

    protected:
        void createAnswer() override;

    private:
        void addTransceiver(
            cricket::MediaType type,
            rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
            bool offerToReceive);
        void updateTransceiver(
            cricket::MediaType type,
            rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
            bool offerToReceive);

        void setAllLocalTracksEnabled(const char* kind, bool enabled);
        void setAllRemoteTracksEnabled(const char* kind, bool enabled);
    };
}

#endif
