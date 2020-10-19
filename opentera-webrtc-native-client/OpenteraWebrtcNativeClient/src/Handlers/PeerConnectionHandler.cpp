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
        const std::function<void(const std::string&)>& onError) :
        m_id(id), m_peerClient(peerClient), m_isCaller(isCaller), m_sendEvent(sendEvent), m_onError(onError)
{
}

PeerConnectionHandler::~PeerConnectionHandler()
{
    m_peerConnection->Close();
}

void PeerConnectionHandler::makePeerCall()
{
    webrtc::PeerConnectionInterface::RTCOfferAnswerOptions options;
    m_peerConnection->CreateOffer(this, options);
}

void PeerConnectionHandler::OnConnectionChange(webrtc::PeerConnectionInterface::PeerConnectionState newState)
{
}

void PeerConnectionHandler::OnIceCandidate(const webrtc::IceCandidateInterface* candidate)
{
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

    std::string sdp;
    desc->ToString(&sdp);

    auto offer = sio::object_message::create();
    offer->get_map()["sdp"] = sio::string_message::create(sdp);

    auto data = sio::object_message::create();
    data->get_map()["toId"] = sio::string_message::create(m_peerClient.id());
    data->get_map()["offer"] = offer;

    if (m_isCaller)
    {
        offer->get_map()["type"] = sio::string_message::create("offer");
        m_sendEvent("call-peer", data);
    }
    else
    {
        offer->get_map()["type"] = sio::string_message::create("answer");
        m_sendEvent("make-peer-call-answer", data);
    }
}

void PeerConnectionHandler::OnFailure(webrtc::RTCError error)
{
    m_onError(error.message());
}
