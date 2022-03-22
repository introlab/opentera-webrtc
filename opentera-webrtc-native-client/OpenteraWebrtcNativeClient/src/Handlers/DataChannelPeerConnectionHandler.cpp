#include <OpenteraWebrtcNativeClient/Handlers/DataChannelPeerConnectionHandler.h>

using namespace opentera;
using namespace std;

static constexpr bool OfferToReceiveVideo = false;
static constexpr bool OfferToReceiveAudio = false;

DataChannelPeerConnectionHandler::DataChannelPeerConnectionHandler(
    string id,
    Client peerClient,
    bool isCaller,
    function<void(const string&, const sio::message::ptr&)> sendEvent,
    function<void(const string&)> onError,
    function<void(const Client&)> onClientConnected,
    function<void(const Client&)> onClientDisconnected,
    string room,
    DataChannelConfiguration dataChannelConfiguration,
    function<void(const Client&)> onDataChannelOpen,
    function<void(const Client&)> onDataChannelClosed,
    function<void(const Client&, const string&)> onDataChannelError,
    function<void(const Client&, const webrtc::DataBuffer& buffer)> onDataChannelMessageBinary,
    function<void(const Client&, const string&)> onDataChannelMessageString)
    : PeerConnectionHandler(
          move(id),
          move(peerClient),
          isCaller,
          OfferToReceiveVideo,
          OfferToReceiveAudio,
          move(sendEvent),
          move(onError),
          move(onClientConnected),
          move(onClientDisconnected)),
      m_room(move(room)),
      m_dataChannelConfiguration(move(dataChannelConfiguration)),
      m_onDataChannelOpen(move(onDataChannelOpen)),
      m_onDataChannelClosed(move(onDataChannelClosed)),
      m_onDataChannelError(move(onDataChannelError)),
      m_onDataChannelMessageBinary(move(onDataChannelMessageBinary)),
      m_onDataChannelMessageString(move(onDataChannelMessageString)),
      m_onDataChannelClosedCalled(true)
{
}

DataChannelPeerConnectionHandler::~DataChannelPeerConnectionHandler()
{
    if (m_dataChannel)
    {
        m_dataChannel->UnregisterObserver();
        m_dataChannel->Close();

        if (!m_onDataChannelClosedCalled)
        {
            m_onDataChannelClosedCalled = true;
            m_onDataChannelClosed(m_peerClient);
        }
    }
}

void DataChannelPeerConnectionHandler::setPeerConnection(
    const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection)
{
    PeerConnectionHandler::setPeerConnection(peerConnection);
    if (m_isCaller)
    {
        auto configuration = static_cast<webrtc::DataChannelInit>(m_dataChannelConfiguration);
        m_dataChannel = m_peerConnection->CreateDataChannel(m_room, &configuration);
        if (m_dataChannel)
        {
            m_dataChannel->RegisterObserver(this);
        }
        else
        {
            m_onError("CreateDataChannel failed");
        }
    }
}

void DataChannelPeerConnectionHandler::send(const webrtc::DataBuffer& buffer)
{
    if (m_dataChannel)
    {
        m_dataChannel->Send(buffer);
    }
}

void DataChannelPeerConnectionHandler::OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> dataChannel)
{
    if (!m_isCaller)
    {
        m_dataChannel = dataChannel;
        m_dataChannel->RegisterObserver(this);
    }
}

void DataChannelPeerConnectionHandler::OnStateChange()
{
    if (m_dataChannel)
    {
        switch (m_dataChannel->state())
        {
            case webrtc::DataChannelInterface::kOpen:
                m_onDataChannelOpen(m_peerClient);
                m_onDataChannelClosedCalled = false;
                break;
            case webrtc::DataChannelInterface::kClosed:
                if (!m_dataChannel->error().ok())
                {
                    m_onDataChannelError(m_peerClient, m_dataChannel->error().message());
                }
                m_onDataChannelClosed(m_peerClient);
                m_onDataChannelClosedCalled = true;
                break;
            default:
                break;
        }
    }
}

void DataChannelPeerConnectionHandler::OnMessage(const webrtc::DataBuffer& buffer)
{
    if (buffer.binary)
    {
        m_onDataChannelMessageBinary(m_peerClient, buffer);
    }
    else if (!buffer.binary)
    {
        m_onDataChannelMessageString(m_peerClient, string(buffer.data.data<char>(), buffer.size()));
    }
}
