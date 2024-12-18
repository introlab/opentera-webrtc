#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>

using namespace opentera;
using namespace std;

class OnlyFailureSetSessionDescriptionObserver : public webrtc::SetSessionDescriptionObserver
{
    function<void(const string&)> m_onError;

public:
    explicit OnlyFailureSetSessionDescriptionObserver(function<void(const string&)> onError) : m_onError(move(onError))
    {
    }

    static OnlyFailureSetSessionDescriptionObserver* Create(function<void(const string&)> onError)
    {
        return new rtc::RefCountedObject<OnlyFailureSetSessionDescriptionObserver>(move(onError));
    }

    void OnSuccess() override {}

    void OnFailure(webrtc::RTCError error) override { m_onError(error.message()); }
};

void CreateSessionDescriptionObserverHelper::OnSuccess(webrtc::SessionDescriptionInterface* desc)
{
    OnCreateSessionDescriptionObserverSuccess(desc);
}

void CreateSessionDescriptionObserverHelper::OnFailure(webrtc::RTCError error)
{
    OnCreateSessionDescriptionObserverFailure(error);
}

void SetSessionDescriptionObserverHelper::OnSuccess()
{
    OnSetSessionDescriptionObserverSuccess();
}

void SetSessionDescriptionObserverHelper::OnFailure(webrtc::RTCError error)
{
    OnSetSessionDescriptionObserverFailure(error);
}

PeerConnectionHandler::PeerConnectionHandler(
    string&& id,
    Client&& peerClient,
    bool isCaller,
    SignalingClient& m_signalingClient,
    function<void(const string&)>&& onError,
    function<void(const Client&)>&& onClientConnected,
    function<void(const Client&)>&& onClientDisconnected,
    function<void(const Client&)>&& onClientConnectionFailed)
    : m_id(move(id)),
      m_peerClient(move(peerClient)),
      m_isCaller(isCaller),
      m_signalingClient(m_signalingClient),
      m_onError(move(onError)),
      m_onClientConnected(move(onClientConnected)),
      m_onClientDisconnected(move(onClientDisconnected)),
      m_onClientConnectionFailed(move(onClientConnectionFailed)),
      m_onClientDisconnectedCalled(true)
{
}

PeerConnectionHandler::~PeerConnectionHandler()
{
    if (m_peerConnection)
    {
        m_onError = [](const string&) {};
        m_onClientConnected = [](const Client&) {};
        m_onClientDisconnected = [](const Client&) {};
        m_peerConnection->Close();

        if (!m_onClientDisconnectedCalled)
        {
            m_onClientDisconnected(m_peerClient);
            m_onClientDisconnectedCalled = true;
        }
    }
}

void PeerConnectionHandler::setPeerConnection(const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection)
{
    m_peerConnection = peerConnection;
}

void PeerConnectionHandler::makePeerCall()
{
    m_peerConnection->CreateOffer(this, webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
}

void PeerConnectionHandler::receivePeerCall(const string& sdp)
{
    webrtc::SdpParseError error;
    auto desc = webrtc::CreateSessionDescription("offer", sdp, &error);
    if (desc)
    {
        m_peerConnection->SetRemoteDescription(this, desc);
    }
    else
    {
        m_onError(error.line + " - " + error.description);
    }
}

void PeerConnectionHandler::receivePeerCallAnswer(const string& sdp)
{
    webrtc::SdpParseError error;
    auto desc = webrtc::CreateSessionDescription("answer", sdp, &error);
    if (desc)
    {
        m_peerConnection->SetRemoteDescription(OnlyFailureSetSessionDescriptionObserver::Create(m_onError), desc);
    }
    else
    {
        m_onError(error.line + " - " + error.description);
    }
}

void PeerConnectionHandler::receiveIceCandidate(const string& sdpMid, int sdpMLineIndex, const string& sdp)
{
    webrtc::SdpParseError error;
    auto candidate = webrtc::CreateIceCandidate(sdpMid, sdpMLineIndex, sdp, &error);
    if (candidate)
    {
        m_peerConnection->AddIceCandidate(candidate);
        delete candidate;
    }
    else if (!error.line.empty())
    {
        m_onError(error.line + " - " + error.description);
    }
}

void PeerConnectionHandler::OnConnectionChange(webrtc::PeerConnectionInterface::PeerConnectionState newState)
{
    switch (newState)
    {
        case webrtc::PeerConnectionInterface::PeerConnectionState::kConnected:
            m_onClientConnected(m_peerClient);
            m_onClientDisconnectedCalled = false;
            break;
        case webrtc::PeerConnectionInterface::PeerConnectionState::kFailed:
            m_onClientConnectionFailed(m_peerClient);
            break;
        case webrtc::PeerConnectionInterface::PeerConnectionState::kDisconnected:
        case webrtc::PeerConnectionInterface::PeerConnectionState::kClosed:
            m_onClientDisconnected(m_peerClient);
            m_onClientDisconnectedCalled = true;
            break;
        default:
            break;
    }
}

void PeerConnectionHandler::OnIceCandidate(const webrtc::IceCandidateInterface* candidate)
{
    string sdp;
    candidate->ToString(&sdp);

    m_signalingClient.sendIceCandidate(candidate->sdp_mid(), candidate->sdp_mline_index(), sdp, m_peerClient.id());
}

void PeerConnectionHandler::OnDataChannel([[maybe_unused]] rtc::scoped_refptr<webrtc::DataChannelInterface> dataChannel)
{
}

void PeerConnectionHandler::OnTrack([[maybe_unused]] rtc::scoped_refptr<webrtc::RtpTransceiverInterface> transceiver) {}

void PeerConnectionHandler::OnRemoveTrack([[maybe_unused]] rtc::scoped_refptr<webrtc::RtpReceiverInterface> receiver) {}

void PeerConnectionHandler::OnSignalingChange([[maybe_unused]] webrtc::PeerConnectionInterface::SignalingState newState)
{
}

void PeerConnectionHandler::OnIceGatheringChange(
    [[maybe_unused]] webrtc::PeerConnectionInterface::IceGatheringState newState)
{
}

void PeerConnectionHandler::OnCreateSessionDescriptionObserverSuccess(webrtc::SessionDescriptionInterface* desc)
{
    m_peerConnection->SetLocalDescription(OnlyFailureSetSessionDescriptionObserver::Create(m_onError), desc);

    string sdp;
    desc->ToString(&sdp);

    if (m_isCaller)
    {
        m_signalingClient.callPeer(m_peerClient.id(), sdp);
    }
    else
    {
        m_signalingClient.makePeerCallAnswer(m_peerClient.id(), sdp);
    }
}

void PeerConnectionHandler::OnCreateSessionDescriptionObserverFailure(webrtc::RTCError error)
{
    m_onError(error.message());
}

void PeerConnectionHandler::OnSetSessionDescriptionObserverSuccess()
{
    if (!m_isCaller)
    {
        createAnswer();
    }
}

void PeerConnectionHandler::OnSetSessionDescriptionObserverFailure(webrtc::RTCError error)
{
    m_onError(error.message());
}

void PeerConnectionHandler::AddRef() const {}

rtc::RefCountReleaseStatus PeerConnectionHandler::Release() const
{
    return rtc::RefCountReleaseStatus::kOtherRefsRemained;
}

void PeerConnectionHandler::createAnswer()
{
    m_peerConnection->CreateAnswer(this, webrtc::PeerConnectionInterface::RTCOfferAnswerOptions());
}

void opentera::setTransceiverDirection(
    const rtc::scoped_refptr<webrtc::RtpTransceiverInterface>& transceiver,
    webrtc::RtpTransceiverDirection direction)
{
    transceiver->SetDirectionWithError(direction);
}
