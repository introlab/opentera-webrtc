#include <OpenteraWebrtcNativeClient/DataChannelClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/DataChannelPeerConnectionHandler.h>

using namespace introlab;
using namespace std;

DataChannelClient::DataChannelClient(SignallingServerConfiguration signallingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration, DataChannelConfiguration dataChannelConfiguration) :
        SignallingClient(move(signallingServerConfiguration), move(webrtcConfiguration)),
        m_dataChannelConfiguration(move(dataChannelConfiguration))
{
}

void DataChannelClient::sendTo(const webrtc::DataBuffer& buffer, const vector<string>& ids)
{
    FunctionTask<void>::callAsync(getInternalClientThread(), [this, buffer, ids]()
    {
        for (const auto& id: ids)
        {
            auto it = m_peerConnectionHandlersById.find(id);
            if (it != m_peerConnectionHandlersById.end())
            {
                dynamic_cast<DataChannelPeerConnectionHandler*>(it->second.get())->send(buffer);
            }
        }
    });
}

void DataChannelClient::sendToAll(const webrtc::DataBuffer& buffer)
{
    FunctionTask<void>::callAsync(getInternalClientThread(), [this, buffer]()
    {
        for (auto& pair : m_peerConnectionHandlersById)
        {
            dynamic_cast<DataChannelPeerConnectionHandler*>(pair.second.get())->send(buffer);
        }
    });
}

unique_ptr<PeerConnectionHandler> DataChannelClient::createPeerConnectionHandler(const string& id,
        const Client& peerClient, bool isCaller)
{
    auto onDataChannelOpen = [this](const Client& client)
    {
        invokeIfCallable(m_onDataChannelOpen, client);
    };
    auto onDataChannelClosed = [this](const Client& client)
    {
        invokeIfCallable(m_onDataChannelClosed, client);
        getOnClientDisconnectedFunction()(client);
    };
    auto onDataChannelError = [this](const Client& client, const string& error)
    {
        invokeIfCallable(m_onDataChannelError, client, error);
    };
    auto onDataChannelMessageBinary = [this](const Client& client, const webrtc::DataBuffer& buffer)
    {
        function<void()> callback = [this, client, buffer]()
        {
            if (m_onDataChannelMessageBinary)
            {
                m_onDataChannelMessageBinary(client, buffer.data.data<uint8_t>(), buffer.size());
            }
        };
        invokeIfCallable(callback);
    };
    auto onDataChannelMessageString = [this](const Client& client, const string& message)
    {
        invokeIfCallable(m_onDataChannelMessageString, client, message);
    };

    return make_unique<DataChannelPeerConnectionHandler>(id, peerClient, isCaller, getSendEventFunction(),
            getOnErrorFunction(), getOnClientConnectedFunction(), getOnClientDisconnectedFunction(),
            m_signallingServerConfiguration.room(), m_dataChannelConfiguration,
            onDataChannelOpen, onDataChannelClosed, onDataChannelError,
            onDataChannelMessageBinary, onDataChannelMessageString);
}
