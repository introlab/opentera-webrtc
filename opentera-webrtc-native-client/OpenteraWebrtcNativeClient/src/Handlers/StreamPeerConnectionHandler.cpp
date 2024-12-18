#include <OpenteraWebrtcNativeClient/Handlers/StreamPeerConnectionHandler.h>

#include <functional>
#include <memory>
#include <utility>

using namespace opentera;
using namespace rtc;
using namespace webrtc;
using namespace std;

StreamPeerConnectionHandler::StreamPeerConnectionHandler(
    string id,
    Client peerClient,
    bool isCaller,
    bool hasOnMixedAudioFrameReceivedCallback,
    SignalingClient& m_signalingClient,
    function<void(const string&)> onError,
    function<void(const Client&)> onClientConnected,
    function<void(const Client&)> onClientDisconnected,
    function<void(const Client&)> onClientConnectionFailed,
    webrtc::scoped_refptr<VideoTrackInterface> videoTrack,
    webrtc::scoped_refptr<AudioTrackInterface> audioTrack,
    function<void(const Client&)> onAddRemoteStream,
    function<void(const Client&)> onRemoveRemoteStream,
    const VideoFrameReceivedCallback& onVideoFrameReceived,
    const EncodedVideoFrameReceivedCallback& onEncodedVideoFrameReceived,
    const AudioFrameReceivedCallback& onAudioFrameReceived)
    : PeerConnectionHandler(
          move(id),
          move(peerClient),
          isCaller,
          m_signalingClient,
          move(onError),
          move(onClientConnected),
          move(onClientDisconnected),
          move(onClientConnectionFailed)),
      m_offerToReceiveVideo(static_cast<bool>(onVideoFrameReceived)),
      m_offerToReceiveAudio(hasOnMixedAudioFrameReceivedCallback || onAudioFrameReceived),
      m_videoTrack(move(videoTrack)),
      m_audioTrack(move(audioTrack)),
      m_onAddRemoteStream(move(onAddRemoteStream)),
      m_onRemoveRemoteStream(move(onRemoveRemoteStream))
{
    if (onVideoFrameReceived)
    {
        m_videoSink = make_unique<VideoSink>([=](const cv::Mat& bgrImg, uint64_t timestampUs)
                                             { onVideoFrameReceived(m_peerClient, bgrImg, timestampUs); });
    }

    if (onEncodedVideoFrameReceived)
    {
        m_encodedVideoSink = make_unique<EncodedVideoSink>(
            [=](const uint8_t* data,
                size_t dataSize,
                VideoCodecType codecType,
                bool isKeyFrame,
                uint32_t width,
                uint32_t height,
                uint64_t timestampUs) {
                onEncodedVideoFrameReceived(
                    m_peerClient,
                    data,
                    dataSize,
                    codecType,
                    isKeyFrame,
                    width,
                    height,
                    timestampUs);
            });
    }

    if (onAudioFrameReceived)
    {
        m_audioSink = make_unique<AudioSink>(
            [=](const void* audioData,
                int bitsPerSample,
                int sampleRate,
                size_t numberOfChannels,
                size_t numberOfFrames) {
                onAudioFrameReceived(
                    m_peerClient,
                    audioData,
                    bitsPerSample,
                    sampleRate,
                    numberOfChannels,
                    numberOfFrames);
            });
    }
}

StreamPeerConnectionHandler::~StreamPeerConnectionHandler()
{
    for (auto& transceiver : m_peerConnection->GetTransceivers())
    {
        transceiver->StopStandard();
    }

    for (auto& track : m_tracks)
    {
        auto videoTrack = dynamic_cast<VideoTrackInterface*>(track.get());
        if (videoTrack != nullptr)
        {
            videoTrack->RemoveSink(m_videoSink.get());
        }

        auto audioTrack = dynamic_cast<AudioTrackInterface*>(track.get());
        if (audioTrack != nullptr)
        {
            audioTrack->RemoveSink(m_audioSink.get());
        }
    }
}

void StreamPeerConnectionHandler::setPeerConnection(
    const webrtc::scoped_refptr<PeerConnectionInterface>& peerConnection)
{
    PeerConnectionHandler::setPeerConnection(peerConnection);

    if (m_isCaller)
    {
        addTransceiver(cricket::MEDIA_TYPE_VIDEO, m_videoTrack, m_offerToReceiveVideo);
        addTransceiver(cricket::MEDIA_TYPE_AUDIO, m_audioTrack, m_offerToReceiveAudio);
    }
}

void StreamPeerConnectionHandler::setAllLocalAudioTracksEnabled(bool enabled)
{
    setAllLocalTracksEnabled(MediaStreamTrackInterface::kAudioKind, enabled);
}

void StreamPeerConnectionHandler::setAllRemoteAudioTracksEnabled(bool enabled)
{
    setAllRemoteTracksEnabled(MediaStreamTrackInterface::kAudioKind, enabled);
}

void StreamPeerConnectionHandler::setAllVideoTracksEnabled(bool enabled)
{
    setAllLocalTracksEnabled(MediaStreamTrackInterface::kVideoKind, enabled);
}

void StreamPeerConnectionHandler::OnTrack(rtc::scoped_refptr<webrtc::RtpTransceiverInterface> transceiver)
{
    if (m_tracks.empty())
    {
        m_onAddRemoteStream(m_peerClient);
    }
    m_tracks.insert(transceiver->receiver()->track());

    auto videoTrack = dynamic_cast<VideoTrackInterface*>(transceiver->receiver()->track().get());
    if (videoTrack != nullptr && m_videoSink != nullptr)
    {
        videoTrack->AddOrUpdateSink(m_videoSink.get(), m_videoSink->wants());
    }
    if (videoTrack != nullptr && m_encodedVideoSink != nullptr)
    {
        videoTrack->GetSource()->AddEncodedSink(m_encodedVideoSink.get());
    }

    auto audioTrack = dynamic_cast<AudioTrackInterface*>(transceiver->receiver()->track().get());
    if (audioTrack != nullptr && m_audioSink != nullptr)
    {
        audioTrack->AddSink(m_audioSink.get());
    }
}

void StreamPeerConnectionHandler::OnRemoveTrack(rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver)
{
    m_tracks.erase(receiver->track());
    if (m_tracks.empty())
    {
        m_onRemoveRemoteStream(m_peerClient);
    }

    auto videoTrack = dynamic_cast<VideoTrackInterface*>(receiver->track().get());
    if (videoTrack != nullptr && m_videoSink != nullptr)
    {
        videoTrack->RemoveSink(m_videoSink.get());
    }
    if (videoTrack != nullptr && m_encodedVideoSink != nullptr)
    {
        videoTrack->GetSource()->RemoveEncodedSink(m_encodedVideoSink.get());
    }

    auto audioTrack = dynamic_cast<AudioTrackInterface*>(receiver->track().get());
    if (audioTrack != nullptr && m_audioSink != nullptr)
    {
        audioTrack->RemoveSink(m_audioSink.get());
    }
}

void StreamPeerConnectionHandler::createAnswer()
{
    updateTransceiver(cricket::MEDIA_TYPE_VIDEO, m_videoTrack, m_offerToReceiveVideo);
    updateTransceiver(cricket::MEDIA_TYPE_AUDIO, m_audioTrack, m_offerToReceiveAudio);
    PeerConnectionHandler::createAnswer();
}

void StreamPeerConnectionHandler::addTransceiver(
    cricket::MediaType type,
    rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
    bool offerToReceive)
{
    RtpTransceiverInit init;

    if (track != nullptr && offerToReceive)
    {
        init.direction = RtpTransceiverDirection::kSendRecv;
        m_peerConnection->AddTransceiver(move(track), init);
    }
    else if (track != nullptr && !offerToReceive)
    {
        init.direction = RtpTransceiverDirection::kSendOnly;
        m_peerConnection->AddTransceiver(move(track), init);
    }
    else if (offerToReceive)
    {
        init.direction = RtpTransceiverDirection::kRecvOnly;
        m_peerConnection->AddTransceiver(type, init);
    }
}

void StreamPeerConnectionHandler::updateTransceiver(
    cricket::MediaType type,
    rtc::scoped_refptr<webrtc::MediaStreamTrackInterface> track,
    bool offerToReceive)
{
    bool isTrackSet = false;
    for (auto& transceiver : m_peerConnection->GetTransceivers())
    {
        if (transceiver->media_type() != type)
        {
            continue;
        }

        if (track != nullptr && offerToReceive)
        {
            setTransceiverDirection(transceiver, RtpTransceiverDirection::kSendRecv);
            transceiver->sender()->SetTrack(track.get());
            isTrackSet = true;
        }
        else if (track != nullptr && !offerToReceive)
        {
            setTransceiverDirection(transceiver, RtpTransceiverDirection::kSendOnly);
            transceiver->sender()->SetTrack(track.get());
            isTrackSet = true;
        }
        else if (!offerToReceive)
        {
            setTransceiverDirection(transceiver, RtpTransceiverDirection::kInactive);
        }
    }

    if (track != nullptr && !isTrackSet)
    {
        RtpTransceiverInit init;
        init.direction = RtpTransceiverDirection::kSendOnly;
        m_peerConnection->AddTransceiver(move(track), init);
    }
}

void StreamPeerConnectionHandler::setAllLocalTracksEnabled(const char* kind, bool enabled)
{
    for (auto& sender : m_peerConnection->GetSenders())
    {
        auto track = sender->track();
        if (track && track->kind() == kind)
        {
            track->set_enabled(enabled);
        }
    }
}

void StreamPeerConnectionHandler::setAllRemoteTracksEnabled(const char* kind, bool enabled)
{
    for (auto& receiver : m_peerConnection->GetReceivers())
    {
        auto track = receiver->track();
        if (track && track->kind() == kind)
        {
            track->set_enabled(enabled);
        }
    }
}
