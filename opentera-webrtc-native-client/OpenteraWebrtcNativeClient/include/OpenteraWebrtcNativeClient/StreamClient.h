#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_STREAM_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_STREAM_CLIENT_H

#include <OpenteraWebrtcNativeClient/VideoSource.h>
#include <OpenteraWebrtcNativeClient/SignallingClient.h>

#include <memory>
#include <OpenteraWebrtcNativeClient/Sinks/VideoSink.h>
#include <OpenteraWebrtcNativeClient/Sinks/AudioSink.h>

namespace introlab
{
    /**
     * @brief a signaling client to join a webrtc room and stream a video source
     */
    class StreamClient: public SignallingClient
    {
        std::shared_ptr<VideoSource> m_videoSource;
        std::shared_ptr<VideoSink> m_videoSink;
        std::shared_ptr<AudioSink> m_audioSink;

        // TODO: figure out what is the base class for an audio source
        // std::shared_ptr<> m_audioSource;

    public:
        StreamClient(const SignallingServerConfiguration& signallingServerConfiguration,
                     const WebrtcConfiguration& webrtcConfiguration,
                     const std::shared_ptr<VideoSource>& videoSource = nullptr,
                     const std::shared_ptr<VideoSink>& videoSink = nullptr,
                     const std::shared_ptr<AudioSink>& audioSink = nullptr);
        ~StreamClient() override = default;

        DECLARE_NOT_COPYABLE(StreamClient);
        DECLARE_NOT_MOVABLE(StreamClient);

    protected:
        std::unique_ptr<PeerConnectionHandler> createPeerConnectionHandler(const std::string& id,
                const Client& peerClient, bool isCaller) override;
    };
}

#endif
