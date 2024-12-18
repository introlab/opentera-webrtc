#include <OpenteraWebrtcNativeClient/StreamClient.h>

using namespace opentera;
using namespace std;

/**
 * @brief Creates a stream client
 *
 * @param signalingServerConfiguration The configuration to connect to the
 * signaling server
 * @param webrtcConfiguration The WebRTC configuration
 * @param videoStreamConfiguration The video stream configuration
 */
StreamClient::StreamClient(
    SignalingServerConfiguration signalingServerConfiguration,
    WebrtcConfiguration webrtcConfiguration,
    VideoStreamConfiguration videoStreamConfiguration)
    : WebrtcClient(move(signalingServerConfiguration), move(webrtcConfiguration), move(videoStreamConfiguration)),
      m_hasOnMixedAudioFrameReceivedCallback(false),
      m_isLocalAudioMuted(false),
      m_isRemoteAudioMuted(false),
      m_isLocalVideoMuted(false)
{
}

/**
 * @brief Creates a stream client
 *
 * @param signalingServerConfiguration The configuration to connect to the
 * signaling server
 * @param webrtcConfiguration The WebRTC configuration
 * @param videoStreamConfiguration The video stream configuration
 * @param videoSource The video source that this client will add to the call
 */
StreamClient::StreamClient(
    SignalingServerConfiguration signalingServerConfiguration,
    WebrtcConfiguration webrtcConfiguration,
    VideoStreamConfiguration videoStreamConfiguration,
    shared_ptr<VideoSource> videoSource)
    : WebrtcClient(move(signalingServerConfiguration), move(webrtcConfiguration), move(videoStreamConfiguration)),
      m_videoSource(move(videoSource)),
      m_hasOnMixedAudioFrameReceivedCallback(false),
      m_isLocalAudioMuted(false),
      m_isRemoteAudioMuted(false),
      m_isLocalVideoMuted(false)
{
}

/**
 * @brief Creates a stream client
 *
 * @param signalingServerConfiguration The configuration to connect to the
 * signaling server
 * @param webrtcConfiguration The WebRTC configuration
 * @param videoStreamConfiguration The video stream configuration
 * @param audioSource The audio source that this client will add to the call
 */
StreamClient::StreamClient(
    SignalingServerConfiguration signalingServerConfiguration,
    WebrtcConfiguration webrtcConfiguration,
    VideoStreamConfiguration videoStreamConfiguration,
    shared_ptr<AudioSource> audioSource)
    : WebrtcClient(move(signalingServerConfiguration), move(webrtcConfiguration), move(videoStreamConfiguration)),
      m_audioSource(move(audioSource)),
      m_hasOnMixedAudioFrameReceivedCallback(false),
      m_isLocalAudioMuted(false),
      m_isRemoteAudioMuted(false),
      m_isLocalVideoMuted(false)
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
 * @param signalingServerConfiguration The configuration to connect to the
 * signaling server
 * @param webrtcConfiguration The WebRTC configuration
 * @param videoStreamConfiguration The video stream configuration
 * @param videoSource The video source that this client will add to the call
 * @param audioSource The audio source that this client will add to the call
 */
StreamClient::StreamClient(
    SignalingServerConfiguration signalingServerConfiguration,
    WebrtcConfiguration webrtcConfiguration,
    VideoStreamConfiguration videoStreamConfiguration,
    shared_ptr<VideoSource> videoSource,
    shared_ptr<AudioSource> audioSource)
    : WebrtcClient(move(signalingServerConfiguration), move(webrtcConfiguration), move(videoStreamConfiguration)),
      m_videoSource(move(videoSource)),
      m_audioSource(move(audioSource)),
      m_hasOnMixedAudioFrameReceivedCallback(false),
      m_isLocalAudioMuted(false),
      m_isRemoteAudioMuted(false),
      m_isLocalVideoMuted(false)
{
    if (m_audioSource != nullptr)
    {
        m_audioProcessing->ApplyConfig(static_cast<webrtc::AudioProcessing::Config>(m_audioSource->configuration()));
        m_audioSource->setAudioDeviceModule(m_audioDeviceModule);
    }
}

StreamClient::~StreamClient()
{
    if (m_audioSource != nullptr)
    {
        m_audioSource->setAudioDeviceModule(nullptr);
    }

    // The Python callback must be destroyed on the Python thread.
    m_audioDeviceModule->setOnMixedAudioFrameReceived(function<void(const void*, int, int, size_t, size_t)>());
}

/**
 * @brief Mutes or unmutes the local audio.
 * @param muted indicates if the local audio is muted or not
 */
void StreamClient::setLocalAudioMuted(bool muted)
{
    callSync(
        getInternalClientThread(),
        [this, muted]()
        {
            this->m_isLocalAudioMuted = muted;
            for (auto& pair : m_peerConnectionHandlersById)
            {
                dynamic_cast<StreamPeerConnectionHandler*>(pair.second.get())->setAllLocalAudioTracksEnabled(!muted);
            }
        });
}

/**
 * @brief Mutes or unmutes the remote audio.
 * @param muted indicates if the remote audio is muted or not
 */
void StreamClient::setRemoteAudioMuted(bool muted)
{
    callSync(
        getInternalClientThread(),
        [this, muted]()
        {
            this->m_isRemoteAudioMuted = muted;
            for (auto& pair : m_peerConnectionHandlersById)
            {
                dynamic_cast<StreamPeerConnectionHandler*>(pair.second.get())->setAllRemoteAudioTracksEnabled(!muted);
            }
        });
}

/**
 * @brief Mutes or unmutes the local video.
 * @param muted indicates if the local video is muted or not
 */
void StreamClient::setLocalVideoMuted(bool muted)
{
    callSync(
        getInternalClientThread(),
        [this, muted]()
        {
            this->m_isLocalVideoMuted = muted;
            for (auto& pair : m_peerConnectionHandlersById)
            {
                dynamic_cast<StreamPeerConnectionHandler*>(pair.second.get())->setAllVideoTracksEnabled(!muted);
            }
        });
}

/**
 * @brief Creates the peer connection handler for this client
 *
 * @param id this peer id
 * @param peerClient this peer client object
 * @param isCaller indicates if this peer initiated the call
 * @return the peer connection handler
 */
unique_ptr<PeerConnectionHandler>
    StreamClient::createPeerConnectionHandler(const string& id, const Client& peerClient, bool isCaller)
{
    // Create a video track if a video source is provided
    rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack = nullptr;
    if (m_videoSource != nullptr)
    {
        videoTrack = m_peerConnectionFactory->CreateVideoTrack(
            rtc::scoped_refptr<webrtc::VideoTrackSourceInterface>(m_videoSource.get()),
            "stream_video");
        videoTrack->set_enabled(!m_isLocalVideoMuted);
    }

    rtc::scoped_refptr<webrtc::AudioTrackInterface> audioTrack = nullptr;
    if (m_audioSource != nullptr)
    {
        audioTrack = m_peerConnectionFactory->CreateAudioTrack("stream_audio", m_audioSource.get());
        audioTrack->set_enabled(!m_isLocalAudioMuted);
    }

    auto onAddRemoteStream = [this](const Client& client) { invokeIfCallable(m_onAddRemoteStream, client); };
    auto onRemoveRemoteStream = [this](const Client& client) { invokeIfCallable(m_onRemoveRemoteStream, client); };

    return make_unique<StreamPeerConnectionHandler>(
        id,
        peerClient,
        isCaller,
        m_hasOnMixedAudioFrameReceivedCallback,
        *m_signalingClient,
        getOnErrorFunction(),
        getOnClientConnectedFunction(),
        getOnClientDisconnectedFunction(),
        getOnClientConnectionFailedFunction(),
        videoTrack,
        audioTrack,
        onAddRemoteStream,
        onRemoveRemoteStream,
        m_onVideoFrameReceived,
        m_onEncodedVideoFrameReceived,
        m_onAudioFrameReceived);
}
