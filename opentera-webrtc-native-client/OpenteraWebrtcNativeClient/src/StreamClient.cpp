#include <OpenteraWebrtcNativeClient/StreamClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/StreamPeerConnectionHandler.h>

using namespace opentera;
using namespace std;

/**
 * @brief Creates a stream client
 *
 * @param signalingServerConfiguration The configuration to connect to the signaling server
 * @param webrtcConfiguration The WebRTC configuration
 */
StreamClient::StreamClient(SignalingServerConfiguration signalingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration) :
        SignalingClient(move(signalingServerConfiguration), move(webrtcConfiguration))
{
}

/**
 * @brief Creates a stream client
 *
 * @param signalingServerConfiguration The configuration to connect to the signaling server
 * @param webrtcConfiguration The WebRTC configuration
 * @param videoSource The video source that this client will add to the call
 */
StreamClient::StreamClient(SignalingServerConfiguration signalingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration,
        shared_ptr<VideoSource> videoSource) :
        SignalingClient(move(signalingServerConfiguration), move(webrtcConfiguration)),
        m_videoSource(move(videoSource))
{
}

/**
 * @brief Creates a stream client
 *
 * @param signalingServerConfiguration The configuration to connect to the signaling server
 * @param webrtcConfiguration The WebRTC configuration
 * @param audioSource The audio source that this client will add to the call
 */
StreamClient::StreamClient(SignalingServerConfiguration signalingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration,
        shared_ptr<AudioSource> audioSource) :
        SignalingClient(move(signalingServerConfiguration), move(webrtcConfiguration)),
        m_audioSource(move(audioSource))
{
    if (m_audioSource != nullptr)
    {
        m_audioProcessing->ApplyConfig(static_cast<webrtc::AudioProcessing::Config>(m_audioSource->configuration()));
        m_audioSource->setAudioDeviceModule(m_audioDeviceModule);
    }
}

/**
 * @brief Creates a stream client
 *
 * @param signalingServerConfiguration The configuration to connect to the signaling server
 * @param webrtcConfiguration The WebRTC configuration
 * @param videoSource The video source that this client will add to the call
 * @param audioSource The audio source that this client will add to the call
 */
StreamClient::StreamClient(SignalingServerConfiguration signalingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration,
        shared_ptr<VideoSource> videoSource,
        shared_ptr<AudioSource> audioSource) :
        SignalingClient(move(signalingServerConfiguration), move(webrtcConfiguration)),
        m_videoSource(move(videoSource)),
        m_audioSource(move(audioSource))
{
    if (m_audioSource != nullptr)
    {
        m_audioProcessing->ApplyConfig(static_cast<webrtc::AudioProcessing::Config>(m_audioSource->configuration()));
        m_audioSource->setAudioDeviceModule(m_audioDeviceModule);
    }
}

StreamClient::~StreamClient()
{
    // The Python callback must be destroyed on the Python thread.
    m_audioDeviceModule->setOnMixedAudioFrameReceived(function<void(const void*, int, int, size_t, size_t)>());
}

/**
 * @brief Create the peer connection handler for this client
 *
 * @param id this peer id
 * @param peerClient this peer client object
 * @param isCaller indicate if this peer initiated the call
 * @return the peer connection handler
 */
unique_ptr<PeerConnectionHandler> StreamClient::createPeerConnectionHandler(const string& id,
                                                                            const Client& peerClient, bool isCaller)
{
    // Create a video track if a video source is provided
    rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack = nullptr;
    if (m_videoSource != nullptr)
    {
        videoTrack = m_peerConnectionFactory->CreateVideoTrack("stream_video", m_videoSource.get());
    }

    rtc::scoped_refptr<webrtc::AudioTrackInterface> audioTrack = nullptr;
    if (m_audioSource != nullptr)
    {
        audioTrack = m_peerConnectionFactory->CreateAudioTrack("stream_audio", m_audioSource.get());
    }

    auto onAddRemoteStream = [this](const Client& client)
    {
        invokeIfCallable(m_onAddRemoteStream, client);
    };
    auto onRemoveRemoteStream = [this](const Client& client)
    {
        invokeIfCallable(m_onRemoveRemoteStream, client);
    };

    return make_unique<StreamPeerConnectionHandler>(
            id,
            peerClient,
            isCaller,
            getSendEventFunction(),
            getOnErrorFunction(),
            getOnClientConnectedFunction(),
            getOnClientDisconnectedFunction(),
            videoTrack,
            audioTrack,
            onAddRemoteStream,
            onRemoveRemoteStream,
            m_onVideoFrameReceived,
            m_onAudioFrameReceived);
}
