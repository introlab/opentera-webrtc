#include <OpenteraWebrtcNativeClient/WebrtcClient.h>
#include <OpenteraWebrtcNativeClient/Codecs/VideoCodecFactories.h>
#include <OpenteraWebrtcNativeClient/Signaling/WebSocketSignalingClient.h>

#include <api/audio_codecs/builtin_audio_decoder_factory.h>
#include <api/audio_codecs/builtin_audio_encoder_factory.h>
#include <api/create_peerconnection_factory.h>

using namespace opentera;
using namespace std;

WebrtcClient::WebrtcClient(
    SignalingServerConfiguration&& signalingServerConfiguration,
    WebrtcConfiguration&& webrtcConfiguration,
    VideoStreamConfiguration&& videoStreamConfiguration)
    : m_webrtcConfiguration(move(webrtcConfiguration)),
      m_destructorCalled(false)
{
    m_signalingClient = make_unique<WebSocketSignalingClient>(signalingServerConfiguration);
    connectSignalingClientCallbacks();

    m_internalClientThread = move(rtc::Thread::Create());
    m_internalClientThread->SetName(signalingServerConfiguration.clientName() + " - internal client", nullptr);
    m_internalClientThread->Start();

    m_networkThread = move(rtc::Thread::CreateWithSocketServer());
    m_networkThread->SetName(signalingServerConfiguration.clientName() + " - network", nullptr);
    m_networkThread->Start();
    m_workerThread = move(rtc::Thread::Create());
    m_workerThread->SetName(signalingServerConfiguration.clientName() + " - worker", nullptr);
    m_workerThread->Start();
    m_signalingThread = move(rtc::Thread::Create());
    m_signalingThread->SetName(signalingServerConfiguration.clientName() + " - signaling", nullptr);
    m_signalingThread->Start();

    m_audioDeviceModule =
        rtc::scoped_refptr<OpenteraAudioDeviceModule>(new rtc::RefCountedObject<OpenteraAudioDeviceModule>);
    m_audioProcessing = webrtc::AudioProcessingBuilder().Create();
    m_peerConnectionFactory = webrtc::CreatePeerConnectionFactory(
        m_networkThread.get(),
        m_workerThread.get(),
        m_signalingThread.get(),
        m_audioDeviceModule,
        webrtc::CreateBuiltinAudioEncoderFactory(),
        webrtc::CreateBuiltinAudioDecoderFactory(),
        createVideoEncoderFactory(videoStreamConfiguration),
        createVideoDecoderFactory(videoStreamConfiguration),
        nullptr,  // Audio mixer,
        m_audioProcessing);

    if (!m_peerConnectionFactory)
    {
        throw runtime_error("CreatePeerConnectionFactory failed");
    }
}

WebrtcClient::~WebrtcClient()
{
    callSync(m_internalClientThread.get(), [this]() { m_destructorCalled = true; });
    closeSync();
}

/**
 * Enable or disable the TLS verification. By default, the TLS verification is
 * enabled.
 * @param isEnabled
 */
void WebrtcClient::setTlsVerificationEnabled(bool isEnabled)
{
    callAsync(
        m_internalClientThread.get(),
        [this, isEnabled]() { m_signalingClient->setTlsVerificationEnabled(isEnabled); });
}

/**
 * @brief Connects the client the signaling server.
 */
void WebrtcClient::connect()
{
    callAsync(
        m_internalClientThread.get(),
        [this]()
        {
            closeAllConnections();
            m_signalingClient->connect();
        });
}

/**
 * @brief Closes all client connections (asynchronous).
 */
void WebrtcClient::close()
{
    callAsync(
        m_internalClientThread.get(),
        [this]()
        {
            closeAllConnections();
            m_signalingClient->close();
        });
}

/**
 * @brief Closes all client connections (synchronous).
 */
void WebrtcClient::closeSync()
{
    callSync(
        m_internalClientThread.get(),
        [this]()
        {
            closeAllConnections();
            m_signalingClient->closeSync();
        });
}

/**
 * @brief Calls all room clients.
 */
void WebrtcClient::callAll()
{
    callAsync(
        m_internalClientThread.get(),
        [this]()
        {
            m_alreadyAcceptedCalls.clear();
            for (const auto& pair : m_roomClientsById)
            {
                m_alreadyAcceptedCalls.emplace_back(pair.first);
            }

            m_signalingClient->callAll();
        });
}

/**
 * @brief Calls the specified clients.
 * @param ids The client ids to call
 */
void WebrtcClient::callIds(const vector<string>& ids)
{
    callAsync(
        m_internalClientThread.get(),
        [this, ids]()
        {
            m_alreadyAcceptedCalls = ids;
            m_signalingClient->callIds(ids);
        });
}

/**
 * @brief Hangs up all clients.
 */
void WebrtcClient::hangUpAll()
{
    callAsync(
        m_internalClientThread.get(),
        [this]()
        {
            closeAllConnections();
            invokeIfCallable(m_onRoomClientsChanged, getRoomClients());
        });
}

/**
 * @brief Closes all room peer connections.
 */
void WebrtcClient::closeAllRoomPeerConnections()
{
    callAsync(m_internalClientThread.get(), [this]() { m_signalingClient->closeAllRoomPeerConnections(); });
}

/**
 * @brief Returns the connected room client ids.
 * @return The connected room client ids
 */
vector<string> WebrtcClient::getConnectedRoomClientIds()
{
    return callSync(
        m_internalClientThread.get(),
        [this]()
        {
            vector<string> ids;
            ids.reserve(m_peerConnectionHandlersById.size());
            for (const auto& pair : m_peerConnectionHandlersById)
            {
                ids.emplace_back(pair.first);
            }
            return ids;
        });
}

/**
 * @brief Returns the room clients
 * @return The room clients
 */
vector<RoomClient> WebrtcClient::getRoomClients()
{
    return callSync(
        m_internalClientThread.get(),
        [this]()
        {
            vector<RoomClient> roomClients;
            roomClients.reserve(m_roomClientsById.size());
            for (const auto& pair : m_roomClientsById)
            {
                auto client = pair.second;
                bool isConnected =
                    m_peerConnectionHandlersById.find(client.id()) != m_peerConnectionHandlersById.end() ||
                    client.id() == id();
                roomClients.emplace_back(client, isConnected);
            }

            return roomClients;
        });
}

function<void(const string&)> WebrtcClient::getOnErrorFunction()
{
    return [this](const string& message) { invokeIfCallable(m_onError, message); };
}

function<void(const Client&)> WebrtcClient::getOnClientConnectedFunction()
{
    return [this](const Client& client) { invokeIfCallable(m_onClientConnected, client); };
}

function<void(const Client&)> WebrtcClient::getOnClientDisconnectedFunction()
{
    return [this](const Client& client)
    {
        callAsync(
            m_internalClientThread.get(),
            [this, client]()
            {
                removeConnection(client.id());
                invokeIfCallable(m_onClientDisconnected, client);
            });
    };
}

function<void(const Client&)> WebrtcClient::getOnClientConnectionFailedFunction()
{
    return [this](const Client& client)
    {
        callAsync(
            m_internalClientThread.get(),
            [this, client]()
            {
                removeConnection(client.id());
                invokeIfCallable(m_onClientConnectionFailed, client);
            });
    };
}

void setOnRoomClientsChanged(const function<void(const vector<Client>&)>& callback);

void WebrtcClient::connectSignalingClientCallbacks()
{
    m_signalingClient->setOnSignalingConnectionOpened([this]() { invokeIfCallable(m_onSignalingConnectionOpened); });
    m_signalingClient->setOnSignalingConnectionClosed([this]() { invokeIfCallable(m_onSignalingConnectionClosed); });
    m_signalingClient->setOnSignalingConnectionError([this](const string& error)
                                                     { invokeIfCallable(m_onSignalingConnectionError, error); });

    m_signalingClient->setOnRoomClientsChanged(
        [this](const vector<Client>& clients)
        {
            if (m_destructorCalled)
            {
                return;
            }

            callAsync(
                m_internalClientThread.get(),
                [this, clients]()
                {
                    m_roomClientsById.clear();
                    for (auto& client : clients)
                    {
                        m_roomClientsById[client.id()] = client;
                    }
                    invokeIfCallable(m_onRoomClientsChanged, getRoomClients());
                });
        });

    m_signalingClient->setMakePeerCall(
        [this](const string& id)
        {
            if (m_destructorCalled)
            {
                return;
            }

            makePeerCall(id);
        });
    m_signalingClient->setReceivePeerCall(
        [this](const string& fromId, const string& sdp)
        {
            if (m_destructorCalled)
            {
                return;
            }

            receivePeerCall(fromId, sdp);
        });
    m_signalingClient->setReceivePeerCallAnswer(
        [this](const string& fromId, const string& sdp)
        {
            if (m_destructorCalled)
            {
                return;
            }

            receivePeerCallAnswer(fromId, sdp);
        });
    m_signalingClient->setReceiveIceCandidate(
        [this](const string& fromId, const string& sdpMid, int sdpMLineIndex, const string& sdp)
        {
            if (m_destructorCalled)
            {
                return;
            }

            receiveIceCandidate(fromId, sdpMid, sdpMLineIndex, sdp);
        });

    m_signalingClient->setOnCallRejected(
        [this](const string& fromId)
        {
            if (m_destructorCalled)
            {
                return;
            }

            callAsync(
                m_internalClientThread.get(),
                [this, fromId]()
                {
                    removeConnection(fromId);
                    auto clientIt = m_roomClientsById.find(fromId);
                    invokeIfCallable(m_onCallRejected, clientIt->second);
                });
        });

    m_signalingClient->setCloseAllPeerConnections(
        [this]()
        {
            if (m_destructorCalled)
            {
                return;
            }

            log("onCloseAllPeerConnectionsRequestReceivedEvent");
            hangUpAll();
        });

    m_signalingClient->setOnError([this](const string& error) { invokeIfCallable(m_onError, error); });
}

void WebrtcClient::makePeerCall(const string& id)
{
    callAsync(
        m_internalClientThread.get(),
        [this, id]()
        {
            log("makePeerCall (to_id=" + id + ")");
            auto clientIt = m_roomClientsById.find(id);
            if (clientIt == m_roomClientsById.end())
            {
                log("makePeerCall failed because " + id + " is not in the room.");
                return;
            }
            if (m_peerConnectionHandlersById.find(id) != m_peerConnectionHandlersById.end())
            {
                log("makePeerCall failed because " + id + " is already connected.");
                return;
            }

            if (!getCallAcceptance(id))
            {
                invokeIfCallable(m_onCallRejected, clientIt->second);
                return;
            }

            m_peerConnectionHandlersById[id] = createConnection(id, clientIt->second, true);
            m_peerConnectionHandlersById[id]->makePeerCall();
        });
}

void WebrtcClient::receivePeerCall(const string& fromId, const string& sdp)
{
    callAsync(
        m_internalClientThread.get(),
        [this, fromId, sdp]()
        {
            log("receivePeerCall (from_id=" + fromId + ")");
            auto fromClientIt = m_roomClientsById.find(fromId);
            if (fromClientIt == m_roomClientsById.end())
            {
                return;
            }

            if (m_peerConnectionHandlersById.find(fromId) != m_peerConnectionHandlersById.end())
            {
                log("receivePeerCall failed because " + fromId + " is already connected.");
                return;
            }
            if (!getCallAcceptance(fromId))
            {
                m_signalingClient->rejectCall(fromId);
                return;
            }

            m_peerConnectionHandlersById[fromId] = createConnection(fromId, fromClientIt->second, false);
            m_peerConnectionHandlersById[fromId]->receivePeerCall(sdp);
        });
}

void WebrtcClient::receivePeerCallAnswer(const string& fromId, const string& sdp)
{
    callAsync(
        m_internalClientThread.get(),
        [this, fromId, sdp]()
        {
            log("receivePeerCallAnswer (from_id=" + fromId + ")");
            auto peerConnectionsHandlerIt = m_peerConnectionHandlersById.find(fromId);
            if (peerConnectionsHandlerIt == m_peerConnectionHandlersById.end())
            {
                log("receivePeerCallAnswer failed because " + fromId + " is not found.");
                return;
            }

            peerConnectionsHandlerIt->second->receivePeerCallAnswer(sdp);
        });
}

void WebrtcClient::receiveIceCandidate(const string& fromId, const string& sdpMid, int sdpMLineIndex, const string& sdp)
{
    callAsync(
        m_internalClientThread.get(),
        [this, fromId, sdpMid, sdpMLineIndex, sdp]()
        {
            log("receiveIceCandidate (from_id=" + fromId + ")");
            auto peerConnectionsHandlerIt = m_peerConnectionHandlersById.find(fromId);
            if (peerConnectionsHandlerIt == m_peerConnectionHandlersById.end())
            {
                log("receivePeerCallAnswer failed because " + fromId + " is already connected.");
                return;
            }

            peerConnectionsHandlerIt->second->receiveIceCandidate(sdpMid, sdpMLineIndex, sdp);
        });
}

void WebrtcClient::closeAllConnections()
{
    callSync(
        m_internalClientThread.get(),
        [this]()
        {
            vector<string> ids(m_peerConnectionHandlersById.size());
            for (const auto& pair : m_peerConnectionHandlersById)
            {
                ids.emplace_back(pair.first);
            }

            for (const auto& id : ids)
            {
                removeConnection(id);
            }
        });
}

bool WebrtcClient::getCallAcceptance(const string& id)
{
    return callSync(
        m_internalClientThread.get(),
        [this, id]()
        {
            if (find(m_alreadyAcceptedCalls.begin(), m_alreadyAcceptedCalls.end(), id) != m_alreadyAcceptedCalls.end())
            {
                log("The call is already accepted (id=" + id + ").");
                return true;
            }

            auto clientIt = m_roomClientsById.find(id);
            if (clientIt == m_roomClientsById.end())
            {
                log("The call is rejected because " + id + " is not in the room.");
                return false;
            }
            else if (!m_callAcceptor)
            {
                log("The call is accepted by default (id=" + id + ").");
                return true;
            }
            else if (m_callAcceptor(clientIt->second))
            {
                log("The call is accepted by the user (id=" + id + ").");
                return true;
            }
            else
            {
                log("The call is rejected by the user (id=" + id + ").");
                return false;
            }
        });
}

unique_ptr<PeerConnectionHandler>
    WebrtcClient::createConnection([[maybe_unused]] const string& peerId, const Client& peerClient, bool isCaller)
{
    return callSync(
        m_internalClientThread.get(),
        [&, this]()
        {
            auto configuration = static_cast<webrtc::PeerConnectionInterface::RTCConfiguration>(m_webrtcConfiguration);
            unique_ptr<PeerConnectionHandler> handler = createPeerConnectionHandler(id(), peerClient, isCaller);
            auto peerConnection = m_peerConnectionFactory->CreatePeerConnectionOrError(
                configuration,
                webrtc::PeerConnectionDependencies(handler.get()));
            if (peerConnection.ok())
            {
                handler->setPeerConnection(peerConnection.MoveValue());
            }
            else
            {
                throw runtime_error(string("Failed to create peer connection: ") + peerConnection.error().message());
            }
            return handler;
        });
}

void WebrtcClient::removeConnection(const string& id)
{
    callSync(
        m_internalClientThread.get(),
        [this, &id]()
        {
            log("removeConnection (" + id + ")");

            auto it = find(m_alreadyAcceptedCalls.begin(), m_alreadyAcceptedCalls.end(), id);
            if (it != m_alreadyAcceptedCalls.end())
            {
                m_alreadyAcceptedCalls.erase(it);
            }

            auto peerConnectionIt = m_peerConnectionHandlersById.find(id);
            if (peerConnectionIt != m_peerConnectionHandlersById.end())
            {
                unique_ptr<PeerConnectionHandler> peerConnectionHandler;
                peerConnectionHandler = move(peerConnectionIt->second);
                m_peerConnectionHandlersById.erase(peerConnectionIt);
            }
        });
}
