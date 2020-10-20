#include <OpenteraWebrtcNativeClient/DataChannelClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/DataChannelPeerConnectionHandler.h>

using namespace introlab;
using namespace std;

DataChannelClient::DataChannelClient(const SignallingServerConfiguration& signallingServerConfiguration,
        const WebrtcConfiguration& webrtcConfiguration, const DataChannelConfiguration& dataChannelConfiguration) :
        SignallingClient(signallingServerConfiguration, webrtcConfiguration),
        m_dataChannelConfiguration(dataChannelConfiguration)
{
}

void DataChannelClient::sendTo(const webrtc::DataBuffer& buffer, const vector<string>& ids)
{
    FunctionTask<void>::callSync(getInternalClientThread(), [this, &buffer, &ids]()
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
    FunctionTask<void>::callSync(getInternalClientThread(), [this, &buffer]()
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
    auto onDataChannelMessageBinary = [this](const Client& client, const uint8_t* data, size_t size)
    {
        invokeIfCallable(m_onDataChannelMessageBinary, client, data, size);
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
