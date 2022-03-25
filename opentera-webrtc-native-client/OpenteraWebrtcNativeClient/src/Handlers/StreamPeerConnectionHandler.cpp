#include <OpenteraWebrtcNativeClient/Handlers/StreamPeerConnectionHandler.h>

#include <algorithm>
#include <utility>

using namespace opentera;
using namespace rtc;
using namespace webrtc;
using namespace std;

static constexpr bool OfferToReceiveVideo = true;
static constexpr bool OfferToReceiveAudio = true;

StreamPeerConnectionHandler::StreamPeerConnectionHandler(
    string id,
    Client peerClient,
    bool isCaller,
    bool hasOnMixedAudioFrameReceivedCallback,
    function<void(const string&, const sio::message::ptr&)> sendEvent,
    function<void(const string&)> onError,
    function<void(const Client&)> onClientConnected,
    function<void(const Client&)> onClientDisconnected,
    scoped_refptr<VideoTrackInterface> videoTrack,
    scoped_refptr<AudioTrackInterface> audioTrack,
    function<void(const Client&)> onAddRemoteStream,
    function<void(const Client&)> onRemoveRemoteStream,
    const VideoFrameReceivedCallback& onVideoFrameReceived,
    const AudioFrameReceivedCallback& onAudioFrameReceived,
    const webrtc::RtpCapabilities& videoCapabilities)
    : PeerConnectionHandler(
          move(id),
          move(peerClient),
          isCaller,
          move(sendEvent),
          move(onError),
          move(onClientConnected),
          move(onClientDisconnected)),
      m_offerToReceiveAudio(hasOnMixedAudioFrameReceivedCallback || onAudioFrameReceived),
      m_offerToReceiveVideo(static_cast<bool>(onVideoFrameReceived)),
      m_videoTrack(move(videoTrack)),
      m_audioTrack(move(audioTrack)),
      m_onAddRemoteStream(move(onAddRemoteStream)),
      m_onRemoveRemoteStream(move(onRemoveRemoteStream)),
      m_videoCapabilities(videoCapabilities)
{
    if (onVideoFrameReceived)
    {
        m_videoSink = make_unique<VideoSink>([=](const cv::Mat& bgrImg, uint64_t timestampUs)
                                             { onVideoFrameReceived(m_peerClient, bgrImg, timestampUs); });
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

void StreamPeerConnectionHandler::setPeerConnection(const scoped_refptr<PeerConnectionInterface>& peerConnection)
{
    PeerConnectionHandler::setPeerConnection(peerConnection);

    if (m_isCaller)
    {
        addTransceiver(cricket::MEDIA_TYPE_VIDEO, m_videoTrack, m_offerToReceiveVideo);
        addTransceiver(cricket::MEDIA_TYPE_AUDIO, m_audioTrack, m_offerToReceiveAudio);
        setVideoCodecPreferences();
    }
}

void StreamPeerConnectionHandler::setAllAudioTracksEnabled(bool enabled)
{
    setAllTracksEnabled(MediaStreamTrackInterface::kAudioKind, enabled);
}

void StreamPeerConnectionHandler::setAllVideoTracksEnabled(bool enabled)
{
    setAllTracksEnabled(MediaStreamTrackInterface::kVideoKind, enabled);
}

void StreamPeerConnectionHandler::OnTrack(rtc::scoped_refptr<webrtc::RtpTransceiverInterface> transceiver)
{
    if (m_tracks.empty())
    {
        m_onAddRemoteStream(m_peerClient);
    }
    m_tracks.insert(transceiver->receiver()->track());

    auto videoTrack = dynamic_cast<VideoTrackInterface*>(transceiver->receiver()->track().get());
    if (videoTrack != nullptr)
    {
        videoTrack->AddOrUpdateSink(m_videoSink.get(), m_videoSink->wants());
    }

    auto audioTrack = dynamic_cast<AudioTrackInterface*>(transceiver->receiver()->track().get());
    if (audioTrack != nullptr)
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
    if (videoTrack != nullptr)
    {
        videoTrack->RemoveSink(m_videoSink.get());
    }

    auto audioTrack = dynamic_cast<AudioTrackInterface*>(receiver->track().get());
    if (audioTrack != nullptr)
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

    setVideoCodecPreferences();
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
            transceiver->sender()->SetTrack(track);
            isTrackSet = true;
        }
        else if (track != nullptr && !offerToReceive)
        {
            setTransceiverDirection(transceiver, RtpTransceiverDirection::kSendOnly);
            transceiver->sender()->SetTrack(track);
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

    setVideoCodecPreferences();
}

RtpCodecCapability
    createRtpCodecCapability(cricket::MediaType kind, string name, int clockRate, map<string, string> parameters)
{
    RtpCodecCapability capability;
    capability.kind = kind;
    capability.name = move(name);
    capability.clock_rate = clockRate;
    capability.parameters.insert(parameters.begin(), parameters.end());

    return capability;
}

void StreamPeerConnectionHandler::setVideoCodecPreferences()
{
#ifdef OPENTERA_WEBRTC_NATIVE_CLIENT_FORCE_H264
    vector<RtpCodecCapability> h264Capabilities;
    copy_if(
        m_videoCapabilities.codecs.begin(),
        m_videoCapabilities.codecs.end(),
        back_inserter(h264Capabilities),
        [](RtpCodecCapability capability) { return capability.name == "H264"; });
    rtc::ArrayView<RtpCodecCapability> capabilitiesArrayView =
        MakeArrayView(h264Capabilities.data(), h264Capabilities.size());

    for (auto& transceiver : m_peerConnection->GetTransceivers())
    {
        if (transceiver->media_type() != cricket::MEDIA_TYPE_VIDEO)
        {
            continue;
        }

        transceiver->SetCodecPreferences(capabilitiesArrayView);
    }
#endif
}

void StreamPeerConnectionHandler::setAllTracksEnabled(const char* kind, bool enabled)
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
