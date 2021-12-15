#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_STREAM_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_STREAM_CLIENT_H

#include <OpenteraWebrtcNativeClient/Sources/AudioSource.h>
#include <OpenteraWebrtcNativeClient/Sources/VideoSource.h>

#include <OpenteraWebrtcNativeClient/SignalingClient.h>

#include <memory>

namespace opentera
{
    /**
     * @brief A signaling client to join a WebRTC room and stream a video source.
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

        bool m_isLocalAudioMuted;
        bool m_isLocalVideoMuted;

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
        ~StreamClient() override;

        DECLARE_NOT_COPYABLE(StreamClient);
        DECLARE_NOT_MOVABLE(StreamClient);

        bool isLocalAudioMuted();
        void muteLocalAudio();
        void unmuteLocalAudio();
        void setLocalAudioMuted(bool muted);

        bool isLocalVideoMuted();
        void muteLocalVideo();
        void unmuteLocalVideo();
        void setLocalVideoMuted(bool muted);

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
        void setOnMixedAudioFrameReceived(const std::function<void(
                const void* audioData,
                int bitsPerSample,
                int sampleRate,
                size_t numberOfChannels,
                size_t numberOfFrames)>& callback);

    protected:
        std::unique_ptr<PeerConnectionHandler> createPeerConnectionHandler(const std::string& id,
                const Client& peerClient, bool isCaller) override;
    };

    /**
     * @brief Indicates if the local audio is muted.
     * @return true if the local audio is muted.
     */
    inline bool StreamClient::isLocalAudioMuted()
    {
        return FunctionTask<bool>::callSync(getInternalClientThread(), [this]()
        {
            return m_isLocalAudioMuted;
        });
    }


    /**
     * @brief Mutes the local audio.
     */
    inline void StreamClient::muteLocalAudio()
    {
        setLocalAudioMuted(true);
    }

    /**
     * @brief Unmutes the local audio.
     */
    inline void StreamClient::unmuteLocalAudio()
    {
        setLocalAudioMuted(false);
    }

    /**
     * @brief Indicates if the local video is muted.
     * @return true if the local audio is muted.
     */
    inline bool StreamClient::isLocalVideoMuted()
    {
        return FunctionTask<bool>::callSync(getInternalClientThread(), [this]()
        {
            return m_isLocalVideoMuted;
        });
    }

    /**
     * @brief Mutes the local video.
     */
    inline void StreamClient::muteLocalVideo()
    {
        setLocalVideoMuted(true);
    }

    /**
     * @brief Unmutes the local video.
     */
    inline void StreamClient::unmuteLocalVideo()
    {
        setLocalVideoMuted(false);
    }

    /**
     * @brief Sets the callback that is called when a stream is added.
     *
     * The callback is called from the internal client thread.
     *
     * @parblock
     * Callback parameters:
     * - client: The client of the stream
     * @endparblock
     *
     * @param callback The callback
     */
    inline void StreamClient::setOnAddRemoteStream(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onAddRemoteStream = callback;
        });
    }

    /**
     * @brief Sets the callback that is called when a stream is removed.
     *
     * The callback is called from the internal client thread.
     *
     * @parblock
     * Callback parameters:
     * - client: The client of the stream
     * @endparblock
     *
     * @param callback The callback
     */
    inline void StreamClient::setOnRemoveRemoteStream(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onRemoveRemoteStream = callback;
        });
    }

    /**
     * @brief Sets the callback that is called when a video stream frame is received.
     *
     * The callback is called from a WebRTC processing thread.
     *
     * @parblock
     * Callback parameters:
     * - client: The client of the stream frame
     * - bgrImg: The BGR frame image
     * - timestampUs The timestamp in microseconds
     * @endparblock
     *
     * @param callback The callback
     */
    inline void StreamClient::setOnVideoFrameReceived(
            const std::function<void(const Client&, const cv::Mat& bgrImg, uint64_t timestampUs)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onVideoFrameReceived = callback;
        });
    }

    /**
     * @brief Sets the callback that is called when an audio stream frame is received.
     *
     * The callback is called from a WebRTC processing thread.
     *
     * @parblock
     * Callback parameters:
     * - client: The client of the stream frame
     * - audioData: The audio data
     * - bitsPerSample: The audio stream sample size (8, 16 or 32 bits)
     * - sampleRate: The audio stream sample rate
     * - numberOfChannels: The audio stream channel count
     * - numberOfFrames: The number of frames
     * @endparblock
     *
     * @param callback The callback
     */
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

    /**
     * @brief Sets the callback that is called when a mixed audio stream frame is received.
     *
     * The callback is called from a WebRTC processing thread.
     *
     * @parblock
     * Callback parameters:
     * - audioData: The audio data
     * - bitsPerSample: The audio stream sample size (8, 16 or 32 bits)
     * - sampleRate: The audio stream sample rate
     * - numberOfChannels: The audio stream channel count
     * - numberOfFrames: The number of frames
     * @endparblock
     *
     * @param callback The callback
     */
    inline void StreamClient::setOnMixedAudioFrameReceived(const std::function<void(
            const void* audioData,
            int bitsPerSample,
            int sampleRate,
            size_t numberOfChannels,
            size_t numberOfFrames)>& callback)
    {
        m_audioDeviceModule->setOnMixedAudioFrameReceived(callback);
    }
}

#endif
