#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_STREAM_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_STREAM_CLIENT_H

#include <OpenteraWebrtcNativeClient/Sources/AudioSource.h>
#include <OpenteraWebrtcNativeClient/Sources/VideoSource.h>

#include <OpenteraWebrtcNativeClient/SignalingClient.h>

#include <memory>

namespace opentera
{
    /**
     * @brief a signaling client to join a webrtc room and stream a video source
     */
    class StreamClient: public SignalingClient
    {
        std::shared_ptr<VideoSource> m_videoSource;
        std::shared_ptr<AudioSource> m_audioSource;

        std::function<void(const Client&)> m_onAddRemoteStream;
        std::function<void(const Client&)> m_onRemoveRemoteStream;
        std::function<void(const Client&, const cv::Mat&, uint64_t)> m_onVideoFrameReceived;
        std::function<void(const Client& client,
                const void* audioData,
                int bitsPerSample,
                int sampleRate,
                size_t numberOfChannels,
                size_t numberOfFrames)> m_onAudioFrameReceived;

    public:
        StreamClient(SignalingServerConfiguration signalingServerConfiguration,
                WebrtcConfiguration webrtcConfiguration);
        StreamClient(SignalingServerConfiguration signalingServerConfiguration,
                WebrtcConfiguration webrtcConfiguration,
                std::shared_ptr<VideoSource> videoSource);
        StreamClient(SignalingServerConfiguration signalingServerConfiguration,
                WebrtcConfiguration webrtcConfiguration,
                std::shared_ptr<AudioSource> audioSource);
        StreamClient(SignalingServerConfiguration signalingServerConfiguration,
                WebrtcConfiguration webrtcConfiguration,
                std::shared_ptr<VideoSource> videoSource,
                std::shared_ptr<AudioSource> audioSource);
        ~StreamClient() override = default;

        DECLARE_NOT_COPYABLE(StreamClient);
        DECLARE_NOT_MOVABLE(StreamClient);

        void setOnAddRemoteStream(const std::function<void(const Client&)>& callback);
        void setOnRemoveRemoteStream(const std::function<void(const Client&)>& callback);
        void setOnVideoFrameReceived(
                const std::function<void(const Client&, const cv::Mat& bgrImg, uint64_t timestampUs)>& callback);
        void setOnAudioFrameReceived(const std::function<void(
                const Client& client,
                const void* audioData,
                int bitsPerSample,
                int sampleRate,
                size_t numberOfChannels,
                size_t numberOfFrames)>& callback);

    protected:
        std::unique_ptr<PeerConnectionHandler> createPeerConnectionHandler(const std::string& id,
                const Client& peerClient, bool isCaller) override;
    };

    inline void StreamClient::setOnAddRemoteStream(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onAddRemoteStream = callback;
        });
    }

    inline void StreamClient::setOnRemoveRemoteStream(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onRemoveRemoteStream = callback;
        });
    }

    inline void StreamClient::setOnVideoFrameReceived(
            const std::function<void(const Client&, const cv::Mat& bgrImg, uint64_t timestampUs)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onVideoFrameReceived = callback;
        });
    }

    inline void StreamClient::setOnAudioFrameReceived(const std::function<void(
            const Client& client,
            const void* audioData,
            int bitsPerSample,
            int sampleRate,
            size_t numberOfChannels,
            size_t numberOfFrames)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onAudioFrameReceived = callback;
        });
    }
}

#endif
