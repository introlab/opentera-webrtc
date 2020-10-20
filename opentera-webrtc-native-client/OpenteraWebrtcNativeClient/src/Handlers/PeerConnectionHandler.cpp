#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>

using namespace introlab;
using namespace std;

class DummySetSessionDescriptionObserver : public webrtc::SetSessionDescriptionObserver
{
public:
    static DummySetSessionDescriptionObserver* Create() {
        return new rtc::RefCountedObject<DummySetSessionDescriptionObserver>();
    }

    void OnSuccess() override {}
    void OnFailure(webrtc::RTCError error) override {}
};

PeerConnectionHandler::PeerConnectionHandler(const string& id,
        const Client& peerClient,
        bool isCaller,
        const function<void(const string&, sio::message::ptr)>& sendEvent,
        const function<void(const string&)>& onError,
        const function<void(const Client&)>& onClientConnected,
        const function<void(const Client&)>& onClientDisconnected) :
        m_id(id), m_peerClient(peerClient), m_isCaller(isCaller), m_sendEvent(sendEvent), m_onError(onError),
        m_onClientConnected(onClientConnected), m_onClientDisconnected(onClientDisconnected)
{
}

PeerConnectionHandler::~PeerConnectionHandler()
{
    if (m_peerConnection)
    {
        m_sendEvent = [](const string&, sio::message::ptr) {};
        m_onError = [](const string&) {};
        m_onClientConnected = [](const Client&) {};
        m_onClientDisconnected = [](const Client&) {};
        m_peerConnection->Close();
    }
}

void PeerConnectionHandler::setPeerConnection(const rtc::scoped_refptr<webrtc::PeerConnectionInterface> &peerConnection)
{
    m_peerConnection = peerConnection;
}

void PeerConnectionHandler::makePeerCall()
{
    webrtc::PeerConnectionInterface::RTCOfferAnswerOptions options;
    m_peerConnection->CreateOffer(this, options);
}

void PeerConnectionHandler::receivePeerCall(const string& sdp)
{
    webrtc::SdpParseError error;
    auto desc = webrtc::CreateSessionDescription("offer", sdp, &error);
    if (desc)
    {
        m_peerConnection->SetRemoteDescription(DummySetSessionDescriptionObserver::Create(), desc);

        webrtc::PeerConnectionInterface::RTCOfferAnswerOptions options;
        m_peerConnection->CreateAnswer(this, options);
    }
    else
    {
        m_onError(error.line + " - " + error.description);
    }
}

void PeerConnectionHandler::receivePeerCallAnswer(const string &sdp)
{
    webrtc::SdpParseError error;
    auto desc = webrtc::CreateSessionDescription("answer", sdp, &error);
    if (desc)
    {
        m_peerConnection->SetRemoteDescription(DummySetSessionDescriptionObserver::Create(), desc);
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
    else
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
            break;

        case webrtc::PeerConnectionInterface::PeerConnectionState::kDisconnected:
        case webrtc::PeerConnectionInterface::PeerConnectionState::kFailed:
        case webrtc::PeerConnectionInterface::PeerConnectionState::kClosed:
            m_onClientDisconnected(m_peerClient);
            break;

        default:
            break;
    }
}

void PeerConnectionHandler::OnIceCandidate(const webrtc::IceCandidateInterface* candidate)
{
    string sdp;
    candidate->ToString(&sdp);

    auto candidateMessage = sio::object_message::create();
    candidateMessage->get_map()["sdpMid"] = sio::string_message::create(candidate->sdp_mid());
    candidateMessage->get_map()["sdpMLineIndex"] = sio::int_message::create(candidate->sdp_mline_index());
    candidateMessage->get_map()["candidate"] = sio::string_message::create(sdp);

    auto data = sio::object_message::create();
    data->get_map()["toId"] = sio::string_message::create(m_peerClient.id());
    data->get_map()["candidate"] = candidateMessage;

    m_sendEvent("send-ice-candidate", data);
}

void PeerConnectionHandler::OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> dataChannel)
{
}

void PeerConnectionHandler::OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream)
{
}

void PeerConnectionHandler::OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream)
{
}

void PeerConnectionHandler::OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState newState)
{
}

void PeerConnectionHandler::OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState newState)
{
}

void PeerConnectionHandler::OnSuccess(webrtc::SessionDescriptionInterface* desc)
{
    m_peerConnection->SetLocalDescription(DummySetSessionDescriptionObserver::Create(), desc);

    string sdp;
    desc->ToString(&sdp);

    auto offer = sio::object_message::create();
    offer->get_map()["sdp"] = sio::string_message::create(sdp);

    auto data = sio::object_message::create();
    data->get_map()["toId"] = sio::string_message::create(m_peerClient.id());

    if (m_isCaller)
    {
        offer->get_map()["type"] = sio::string_message::create("offer");
        data->get_map()["offer"] = offer;
        m_sendEvent("call-peer", data);
    }
    else
    {
        offer->get_map()["type"] = sio::string_message::create("answer");
        data->get_map()["answer"] = offer;
        m_sendEvent("make-peer-call-answer", data);
    }
}

void PeerConnectionHandler::OnFailure(webrtc::RTCError error)
{
    m_onError(error.message());
}

void PeerConnectionHandler::AddRef() const
{
}

rtc::RefCountReleaseStatus PeerConnectionHandler::Release() const
{
    return rtc::RefCountReleaseStatus::kOtherRefsRemained;
}
