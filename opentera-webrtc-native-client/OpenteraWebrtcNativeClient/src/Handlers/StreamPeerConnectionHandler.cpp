#include <OpenteraWebrtcNativeClient/Handlers/StreamPeerConnectionHandler.h>

#include <utility>

using namespace introlab;
using namespace rtc;
using namespace webrtc;
using namespace std;

StreamPeerConnectionHandler::StreamPeerConnectionHandler(
    string id,
    Client peerClient,
    bool isCaller,
    function<void(const string&, const sio::message::ptr&)> sendEvent,
    function<void(const string&)> onError,
    function<void(const Client&)> onClientConnected,
    function<void(const Client&)> onClientDisconnected,
    scoped_refptr<VideoTrackInterface>  videoTrack,
    scoped_refptr<AudioTrackInterface> audioTrack,
    function<void(const Client&)> onAddRemoteStream,
    function<void(const Client&)> onRemoveRemoteStream,
    const function<void(const Client&, const cv::Mat&, uint64_t)>& onVideoFrameReceived,
    const function<void(const Client&, const void*, int, int, size_t, size_t)>& onAudioFrameReceived) :
    PeerConnectionHandler(move(id), move(peerClient), isCaller, move(sendEvent), move(onError), move(onClientConnected), move(onClientDisconnected)),
    m_videoTrack(move(videoTrack)),
    m_audioTrack(move(audioTrack)),
    m_onAddRemoteStream(move(onAddRemoteStream)),
    m_onRemoveRemoteStream(move(onRemoveRemoteStream))
{
    if (onVideoFrameReceived)
    {
        m_videoSink = make_unique<VideoSink>([=](const cv::Mat& bgrImg, uint64_t timestampUs)
        {
            onVideoFrameReceived(m_peerClient, bgrImg, timestampUs);
        });
    }

    if (onAudioFrameReceived)
    {
        m_audioSink = make_unique<AudioSink>([=](const void* audioData,
                int bitsPerSample,
                int sampleRate,
                size_t numberOfChannels,
                size_t numberOfFrames)
        {
            onAudioFrameReceived(m_peerClient, audioData, bitsPerSample, sampleRate, numberOfChannels, numberOfFrames);
        });
    }
}

StreamPeerConnectionHandler::~StreamPeerConnectionHandler()
{
    for (auto& stream : m_streams)
    {
        VideoTrackVector videoTracks = stream->GetVideoTracks();
        if (!videoTracks.empty() && m_videoSink != nullptr)
        {
            videoTracks.front()->RemoveSink(m_videoSink.get());
        }

        AudioTrackVector audioTracks = stream->GetAudioTracks();
        if (!audioTracks.empty() && m_audioSink != nullptr)
        {
            audioTracks.front()->RemoveSink(m_audioSink.get());
        }
    }
}

void StreamPeerConnectionHandler::setPeerConnection(
        const scoped_refptr<PeerConnectionInterface>& peerConnection)
{
    if (m_videoTrack != nullptr)
    {
        peerConnection->AddTrack(m_videoTrack, vector<string>());
    }
    if (m_audioTrack != nullptr)
    {
        peerConnection->AddTrack(m_audioTrack, vector<string>());
    }

    PeerConnectionHandler::setPeerConnection(peerConnection);
}

void StreamPeerConnectionHandler::OnAddStream(scoped_refptr<MediaStreamInterface> stream)
{
    m_onAddRemoteStream(m_peerClient);
    m_streams.insert(stream);

    VideoTrackVector videoTracks = stream->GetVideoTracks();
    if (!videoTracks.empty() && m_videoSink != nullptr)
    {
        videoTracks.front()->AddOrUpdateSink(m_videoSink.get(), m_videoSink->wants());
    }

    AudioTrackVector audioTracks = stream->GetAudioTracks();
    if (!audioTracks.empty() && m_audioSink != nullptr)
    {
        audioTracks.front()->AddSink(m_audioSink.get());
    }
}

void StreamPeerConnectionHandler::OnRemoveStream(scoped_refptr<MediaStreamInterface> stream)
{
    m_onRemoveRemoteStream(m_peerClient);
    m_streams.erase(stream);

    VideoTrackVector videoTracks = stream->GetVideoTracks();
    if (!videoTracks.empty() && m_videoSink != nullptr)
    {
        videoTracks.front()->RemoveSink(m_videoSink.get());
    }

    AudioTrackVector audioTracks = stream->GetAudioTracks();
    if (!audioTracks.empty() && m_audioSink != nullptr)
    {
        audioTracks.front()->RemoveSink(m_audioSink.get());
    }
}
