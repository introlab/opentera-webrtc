#include <OpenteraWebrtcNativeClient/DataChannelClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/DataChannelPeerConnectionHandler.h>

using namespace introlab;
using namespace std;

DataChannelClient::DataChannelClient(const string& url, const string& clientName, const sio::message::ptr& clientData,
        const string& room, const string& password, const vector<IceServer>& iceServers) :
        SignalingClient(url, clientName, clientData, room, password, iceServers)
{
}

DataChannelClient::~DataChannelClient()
{
}

void DataChannelClient::sendTo(const uint8_t* data, size_t size, const vector<string>& ids)
{
    sendTo(webrtc::DataBuffer(rtc::CopyOnWriteBuffer(data, size), true), ids);
}

void DataChannelClient::sendTo(const string& message, const vector<string>& ids)
{
    sendTo(webrtc::DataBuffer(message), ids);
}

void DataChannelClient::sendToAll(const uint8_t* data, size_t size)
{
    sendToAll(webrtc::DataBuffer(rtc::CopyOnWriteBuffer(data, size), true));
}

void DataChannelClient::sendToAll(const string& message)
{
    sendToAll(webrtc::DataBuffer(message));
}

void DataChannelClient::sendTo(const webrtc::DataBuffer& buffer, const vector<string>& ids)
{
    lock_guard<recursive_mutex> lock(m_peerConnectionMutex);
    for (const auto& id: ids)
    {
        auto it = m_peerConnectionsHandlerById.find(id);
        if (it != m_peerConnectionsHandlerById.end())
        {
            static_cast<DataChannelPeerConnectionHandler*>(it->second.get())->send(buffer);
        }
    }
}

void DataChannelClient::sendToAll(const webrtc::DataBuffer& buffer)
{
    lock_guard<recursive_mutex> lock(m_peerConnectionMutex);
    for (auto& pair : m_peerConnectionsHandlerById)
    {
        static_cast<DataChannelPeerConnectionHandler*>(pair.second.get())->send(buffer);
    }
}

unique_ptr<PeerConnectionHandler> DataChannelClient::createPeerConnectionHandler(const std::string& id,
        const Client& peerClient, bool isCaller)
{
    auto onDataChannelOpen = [this](const Client& client)
    {
        invokeIfCallable(m_onDataChannelOpen, client);
    };
    auto onDataChannelClosed = [this](const Client& client)
    {
        invokeIfCallable(m_onDataChannelClosed, client);
    };
    auto onDataChannelError = [this](const Client& client, const std::string& error)
    {
        invokeIfCallable(m_onDataChannelError, client, error);
    };
    auto onDataChannelMessageBinary = [this](const Client& client, const uint8_t* data, std::size_t size)
    {
        invokeIfCallable(m_onDataChannelMessageBinary, client, data, size);
    };
    auto onDataChannelMessageString = [this](const Client& client, const std::string& message)
    {
        invokeIfCallable(m_onDataChannelMessageString, client, message);
    };

    return make_unique<DataChannelPeerConnectionHandler>(id, peerClient, isCaller, getSendEventFunction(),
            getOnErrorFunction(), onDataChannelOpen, onDataChannelClosed, onDataChannelError,
            onDataChannelMessageBinary, onDataChannelMessageString);
}
