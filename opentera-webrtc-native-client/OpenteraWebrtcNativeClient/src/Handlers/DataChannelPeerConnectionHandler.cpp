#include <OpenteraWebrtcNativeClient/Handlers/DataChannelPeerConnectionHandler.h>

using namespace introlab;
using namespace std;

DataChannelPeerConnectionHandler::DataChannelPeerConnectionHandler(const string& id,
        const Client& peerClient,
        bool isCaller,
        const function<void(const string&, sio::message::ptr)>& sendEvent,
        const function<void(const string&)>& onError,
        const function<void(const Client&)>& onClientConnected,
        const function<void(const Client&)>& onClientDisconnected,
        const string& room,
        const DataChannelConfiguration& dataChannelConfiguration,
        const function<void(const Client&)>& onDataChannelOpen,
        const function<void(const Client&)>& onDataChannelClosed,
        const function<void(const Client&, const string&)>& onDataChannelError,
        const function<void(const Client&, const uint8_t*, size_t)>& onDataChannelMessageBinary,
        const function<void(const Client&, const string&)>& onDataChannelMessageString) :
        PeerConnectionHandler(id, peerClient, isCaller, sendEvent, onError, onClientConnected, onClientDisconnected),
        m_room(room), m_dataChannelConfiguration(dataChannelConfiguration),
        m_onDataChannelOpen(onDataChannelOpen), m_onDataChannelClosed(onDataChannelClosed),
        m_onDataChannelError(onDataChannelError), m_onDataChannelMessageBinary(onDataChannelMessageBinary),
        m_onDataChannelMessageString(onDataChannelMessageString)
{
}

DataChannelPeerConnectionHandler::~DataChannelPeerConnectionHandler()
{
    if (m_dataChannel)
    {
        m_dataChannel->UnregisterObserver();
        m_dataChannel->Close();
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
                break;
            case webrtc::DataChannelInterface::kClosed:
                if (!m_dataChannel->error().ok())
                {
                    m_onDataChannelError(m_peerClient, m_dataChannel->error().message());
                }
                m_onDataChannelClosed(m_peerClient);
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
        m_onDataChannelMessageBinary(m_peerClient, buffer.data.data<uint8_t>(), buffer.size());
    }
    else if (!buffer.binary)
    {
        m_onDataChannelMessageString(m_peerClient, string(buffer.data.data<char>(), buffer.size()));
    }
}
