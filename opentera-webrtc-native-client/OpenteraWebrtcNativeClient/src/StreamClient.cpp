#include <OpenteraWebrtcNativeClient/StreamClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/StreamPeerConnectionHandler.h>

using namespace introlab;
using namespace std;

/**
 * @brief construct a video stream client
 *
 * @param signallingServerConfiguration configuration to connect to the signaling server
 * @param webrtcConfiguration webrtc configuration
 * @param videoSource the video source that this client will add to the call
 * @param videoSink the video sink to attach to the received stream
 * @param audioSink the audio sink to attach to the received stream
 */
StreamClient::StreamClient(
        SignallingServerConfiguration signallingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration,
        shared_ptr<VideoSource> videoSource,
        shared_ptr<VideoSink> videoSink,
        shared_ptr<AudioSink> audioSink) :
        SignallingClient(move(signallingServerConfiguration), move(webrtcConfiguration)),
        m_videoSource(move(videoSource)),
        m_videoSink(move(videoSink)),
        m_audioSink(move(audioSink))
{

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
        videoTrack = m_peerConnectionFactory->CreateVideoTrack("stream", m_videoSource.get());
    }

    rtc::scoped_refptr<webrtc::AudioTrackInterface> audioTrack = nullptr;
    // TODO: create the audio track if an audio source is provided

    return make_unique<StreamPeerConnectionHandler>(
            id,
            peerClient,
            isCaller,
            getSendEventFunction(),
            getOnErrorFunction(),
            getOnClientConnectedFunction(),
            getOnClientDisconnectedFunction(),
            videoTrack,
            m_videoSink,
            audioTrack,
            m_audioSink);
}
