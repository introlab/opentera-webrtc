#include <OpenteraWebrtcNativeClient/DataChannelClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/DataChannelPeerConnectionHandler.h>

using namespace opentera;
using namespace std;

/**
 * @brief Creates a data channel client with the specified configurations.
 *
 * @param signalingServerConfiguration The signaling server configuration
 * @param webrtcConfiguration The WebRTC configuration
 * @param dataChannelConfiguration The data channel configuration
 */
DataChannelClient::DataChannelClient(
    SignalingServerConfiguration signalingServerConfiguration,
    WebrtcConfiguration webrtcConfiguration,
    DataChannelConfiguration dataChannelConfiguration)
    : WebrtcClient(move(signalingServerConfiguration), move(webrtcConfiguration), VideoStreamConfiguration::create()),
      m_dataChannelConfiguration(move(dataChannelConfiguration))
{
}

bool DataChannelClient::sendTo(const webrtc::DataBuffer& buffer, const vector<string>& ids)
{
    return callSync(
        getInternalClientThread(),
        [this, buffer, ids]()
        {
            bool ok = true;
            for (const auto& id : ids)
            {
                auto it = m_peerConnectionHandlersById.find(id);
                if (it != m_peerConnectionHandlersById.end())
                {
                    ok = ok && dynamic_cast<DataChannelPeerConnectionHandler*>(it->second.get())->send(buffer);
                }
            }
            return ok;
        });
}

bool DataChannelClient::sendToAll(const webrtc::DataBuffer& buffer)
{
    return callSync(
        getInternalClientThread(),
        [this, buffer]()
        {
            bool ok = true;
            for (auto& pair : m_peerConnectionHandlersById)
            {
                ok = ok && dynamic_cast<DataChannelPeerConnectionHandler*>(pair.second.get())->send(buffer);
            }
            return ok;
        });
}

unique_ptr<PeerConnectionHandler>
    DataChannelClient::createPeerConnectionHandler(const string& id, const Client& peerClient, bool isCaller)
{
    auto onDataChannelOpen = [this](const Client& client) { invokeIfCallable(m_onDataChannelOpened, client); };
    auto onDataChannelClosed = [this](const Client& client)
    {
        invokeIfCallable(m_onDataChannelClosed, client);
        getOnClientDisconnectedFunction()(client);
    };
    auto onDataChannelError = [this](const Client& client, const string& error)
    { invokeIfCallable(m_onDataChannelError, client, error); };
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
    { invokeIfCallable(m_onDataChannelMessageString, client, message); };

    return make_unique<DataChannelPeerConnectionHandler>(
        id,
        peerClient,
        isCaller,
        *m_signalingClient,
        getOnErrorFunction(),
        getOnClientConnectedFunction(),
        getOnClientDisconnectedFunction(),
        getOnClientConnectionFailedFunction(),
        m_dataChannelConfiguration,
        onDataChannelOpen,
        onDataChannelClosed,
        onDataChannelError,
        onDataChannelMessageBinary,
        onDataChannelMessageString);
}
