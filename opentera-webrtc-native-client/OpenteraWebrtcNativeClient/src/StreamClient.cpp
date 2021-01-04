#include <OpenteraWebrtcNativeClient/StreamClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/StreamPeerConnectionHandler.h>

using namespace introlab;
using namespace std;

/**
 * @brief construct a video stream client
 *
 * @param signallingServerConfiguration configuration to connect to the signaling server
 * @param webrtcConfiguration webrtc configuration
 */
StreamClient::StreamClient(SignallingServerConfiguration signallingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration) :
        SignallingClient(move(signallingServerConfiguration), move(webrtcConfiguration))
{
    initializeInternalStreamThread();
}

/**
 * @brief construct a video stream client
 *
 * @param signallingServerConfiguration configuration to connect to the signaling server
 * @param webrtcConfiguration webrtc configuration
 * @param videoSource the video source that this client will add to the call
 */
StreamClient::StreamClient(SignallingServerConfiguration signallingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration,
        shared_ptr<VideoSource> videoSource) :
        SignallingClient(move(signallingServerConfiguration), move(webrtcConfiguration)),
        m_videoSource(move(videoSource))
{
    initializeInternalStreamThread();
}

/**
 * @brief construct a video stream client
 *
 * @param signallingServerConfiguration configuration to connect to the signaling server
 * @param webrtcConfiguration webrtc configuration
 * @param audioSource the audio source that this client will add to the call
 */
StreamClient::StreamClient(SignallingServerConfiguration signallingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration,
        shared_ptr<AudioSource> audioSource) :
        SignallingClient(move(signallingServerConfiguration), move(webrtcConfiguration)),
        m_audioSource(move(audioSource))
{
    initializeInternalStreamThread();
}

/**
 * @brief construct a video stream client
 *
 * @param signallingServerConfiguration configuration to connect to the signaling server
 * @param webrtcConfiguration webrtc configuration
 * @param videoSource the video source that this client will add to the call
 * @param audioSource the audio source that this client will add to the call
 */
StreamClient::StreamClient(SignallingServerConfiguration signallingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration,
        shared_ptr<VideoSource> videoSource,
        shared_ptr<AudioSource> audioSource) :
        SignallingClient(move(signallingServerConfiguration), move(webrtcConfiguration)),
        m_videoSource(move(videoSource)),
        m_audioSource(move(audioSource))
{
    initializeInternalStreamThread();
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

void StreamClient::initializeInternalStreamThread()
{
    m_internalStreamThread = move(rtc::Thread::Create());
    m_internalStreamThread->SetName(m_signallingServerConfiguration.clientName() + " - internal stream", nullptr);
    m_internalStreamThread->Start();
}
