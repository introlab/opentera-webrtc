#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_STREAM_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_STREAM_CLIENT_H

#include <OpenteraWebrtcNativeClient/Sources/AudioSource.h>
#include <OpenteraWebrtcNativeClient/Sources/VideoSource.h>
#include <OpenteraWebrtcNativeClient/Handlers/StreamPeerConnectionHandler.h>

#include <OpenteraWebrtcNativeClient/WebrtcClient.h>

#include <memory>

namespace opentera
{
    /**
     * @brief A signaling client to join a WebRTC room and stream a video source.
     */
    class StreamClient : public WebrtcClient
    {
        std::shared_ptr<VideoSource> m_videoSource;
        std::shared_ptr<AudioSource> m_audioSource;

        bool m_hasOnMixedAudioFrameReceivedCallback;
        std::function<void(const Client&)> m_onAddRemoteStream;
        std::function<void(const Client&)> m_onRemoveRemoteStream;
        VideoFrameReceivedCallback m_onVideoFrameReceived;
        EncodedVideoFrameReceivedCallback m_onEncodedVideoFrameReceived;
        AudioFrameReceivedCallback m_onAudioFrameReceived;

        bool m_isLocalAudioMuted;
        bool m_isRemoteAudioMuted;
        bool m_isLocalVideoMuted;

    public:
        StreamClient(
            SignalingServerConfiguration signalingServerConfiguration,
            WebrtcConfiguration webrtcConfiguration,
            VideoStreamConfiguration videoStreamConfiguration);
        StreamClient(
            SignalingServerConfiguration signalingServerConfiguration,
            WebrtcConfiguration webrtcConfiguration,
            VideoStreamConfiguration videoStreamConfiguration,
            std::shared_ptr<VideoSource> videoSource);
        StreamClient(
            SignalingServerConfiguration signalingServerConfiguration,
            WebrtcConfiguration webrtcConfiguration,
            VideoStreamConfiguration videoStreamConfiguration,
            std::shared_ptr<AudioSource> audioSource);
        StreamClient(
            SignalingServerConfiguration signalingServerConfiguration,
            WebrtcConfiguration webrtcConfiguration,
            VideoStreamConfiguration videoStreamConfiguration,
            std::shared_ptr<VideoSource> videoSource,
            std::shared_ptr<AudioSource> audioSource);
        ~StreamClient() override;

        DECLARE_NOT_COPYABLE(StreamClient);
        DECLARE_NOT_MOVABLE(StreamClient);

        bool isLocalAudioMuted();
        void muteLocalAudio();
        void unmuteLocalAudio();
        void setLocalAudioMuted(bool muted);

        bool isRemoteAudioMuted();
        void muteRemoteAudio();
        void unmuteRemoteAudio();
        void setRemoteAudioMuted(bool muted);

        bool isLocalVideoMuted();
        void muteLocalVideo();
        void unmuteLocalVideo();
        void setLocalVideoMuted(bool muted);

        void setOnAddRemoteStream(const std::function<void(const Client&)>& callback);
        void setOnRemoveRemoteStream(const std::function<void(const Client&)>& callback);
        void setOnVideoFrameReceived(const VideoFrameReceivedCallback& callback);
        void setOnEncodedVideoFrameReceived(const EncodedVideoFrameReceivedCallback& callback);
        void setOnAudioFrameReceived(const AudioFrameReceivedCallback& callback);
        void setOnMixedAudioFrameReceived(const AudioSinkCallback& callback);

    protected:
        std::unique_ptr<PeerConnectionHandler>
            createPeerConnectionHandler(const std::string& id, const Client& peerClient, bool isCaller) override;
    };

    /**
     * @brief Indicates if the local audio is muted.
     * @return true if the local audio is muted.
     */
    inline bool StreamClient::isLocalAudioMuted()
    {
        return callSync(getInternalClientThread(), [this]() { return m_isLocalAudioMuted; });
    }


    /**
     * @brief Mutes the local audio.
     */
    inline void StreamClient::muteLocalAudio() { setLocalAudioMuted(true); }

    /**
     * @brief Unmutes the local audio.
     */
    inline void StreamClient::unmuteLocalAudio() { setLocalAudioMuted(false); }

    /**
     * @brief Indicates if the remote audio is muted.
     * @return true if the remote audio is muted.
     */
    inline bool StreamClient::isRemoteAudioMuted()
    {
        return callSync(getInternalClientThread(), [this]() { return m_isRemoteAudioMuted; });
    }

    /**
     * @brief Mutes the remote audio.
     */
    inline void StreamClient::muteRemoteAudio() { setRemoteAudioMuted(true); }

    /**
     * @brief Unmutes the remote audio.
     */
    inline void StreamClient::unmuteRemoteAudio() { setRemoteAudioMuted(false); }

    /**
     * @brief Indicates if the local video is muted.
     * @return true if the local audio is muted.
     */
    inline bool StreamClient::isLocalVideoMuted()
    {
        return callSync(getInternalClientThread(), [this]() { return m_isLocalVideoMuted; });
    }

    /**
     * @brief Mutes the local video.
     */
    inline void StreamClient::muteLocalVideo() { setLocalVideoMuted(true); }

    /**
     * @brief Unmutes the local video.
     */
    inline void StreamClient::unmuteLocalVideo() { setLocalVideoMuted(false); }

    /**
     * @brief Sets the callback that is called when a stream is added.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     *  - client: The client of the stream
     * @endparblock
     *
     * @param callback The callback
     */
    inline void StreamClient::setOnAddRemoteStream(const std::function<void(const Client&)>& callback)
    {
        callSync(getInternalClientThread(), [this, &callback]() { m_onAddRemoteStream = callback; });
    }

    /**
     * @brief Sets the callback that is called when a stream is removed.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     *  - client: The client of the stream
     * @endparblock
     *
     * @param callback The callback
     */
    inline void StreamClient::setOnRemoveRemoteStream(const std::function<void(const Client&)>& callback)
    {
        callSync(getInternalClientThread(), [this, &callback]() { m_onRemoveRemoteStream = callback; });
    }

    /**
     * @brief Sets the callback that is called when a video stream frame is received.
     *
     * The callback is called from a WebRTC processing thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     *  - client: The client of the stream frame
     *  - bgrImg: The BGR frame image
     *  - timestampUs The timestamp in microseconds
     * @endparblock
     *
     * @param callback The callback
     */
    inline void StreamClient::setOnVideoFrameReceived(const VideoFrameReceivedCallback& callback)
    {
        callSync(getInternalClientThread(), [this, &callback]() { m_onVideoFrameReceived = callback; });
    }

    /**
     * @brief Sets the callback that is called when an encoded video stream frame is received.
     *
     * The callback is called from a WebRTC processing thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     *  - client: The client of the stream frame
     *  - data: The binary data
     *  - dataSize: The data size
     *  - codecType: The codec type
     *  - isKeyFrame: Indicates if it is a key frame
     *  - width: The frame width if it is a key frame
     *  - height: The frame height if it is a key frame
     *  - timestampUs The timestamp in microseconds
     * @endparblock
     *
     * @param callback The callback
     */
    inline void StreamClient::setOnEncodedVideoFrameReceived(const EncodedVideoFrameReceivedCallback& callback)
    {
        callSync(getInternalClientThread(), [this, &callback]() { m_onEncodedVideoFrameReceived = callback; });
    }

    /**
     * @brief Sets the callback that is called when an audio stream frame is received.
     *
     * The callback is called from a WebRTC processing thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     *  - client: The client of the stream frame
     *  - audioData: The audio data
     *  - bitsPerSample: The audio stream sample size (8, 16 or 32 bits)
     *  - sampleRate: The audio stream sample rate
     *  - numberOfChannels: The audio stream channel count
     *  - numberOfFrames: The number of frames
     * @endparblock
     *
     * @param callback The callback
     */
    inline void StreamClient::setOnAudioFrameReceived(const AudioFrameReceivedCallback& callback)
    {
        callSync(getInternalClientThread(), [this, &callback]() { m_onAudioFrameReceived = callback; });
    }

    /**
     * @brief Sets the callback that is called when a mixed audio stream frame is received.
     *
     * The callback is called from a WebRTC processing thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     *  - audioData: The audio data
     *  - bitsPerSample: The audio stream sample size (8, 16 or 32 bits)
     *  - sampleRate: The audio stream sample rate
     *  - numberOfChannels: The audio stream channel count
     *  - numberOfFrames: The number of frames
     * @endparblock
     *
     * @param callback The callback
     */
    inline void StreamClient::setOnMixedAudioFrameReceived(const AudioSinkCallback& callback)
    {
        bool hasOnMixedAudioFrameReceivedCallback(callback);
        callSync(
            getInternalClientThread(),
            [this, hasOnMixedAudioFrameReceivedCallback]()
            { m_hasOnMixedAudioFrameReceivedCallback = hasOnMixedAudioFrameReceivedCallback; });

        m_audioDeviceModule->setOnMixedAudioFrameReceived(callback);
    }
}

#endif
