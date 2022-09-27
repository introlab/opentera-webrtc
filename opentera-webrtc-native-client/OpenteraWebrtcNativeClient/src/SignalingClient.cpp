#include <OpenteraWebrtcNativeClient/SignalingClient.h>

#include <api/audio_codecs/builtin_audio_decoder_factory.h>
#include <api/audio_codecs/builtin_audio_encoder_factory.h>
#include <api/create_peerconnection_factory.h>
#include <api/video_codecs/builtin_video_decoder_factory.h>
#include <api/video_codecs/builtin_video_encoder_factory.h>

using namespace opentera;
using namespace std;

#define SIO_MESSAGE_CHECK_RETURN(condition, message)                                                                   \
    if ((condition))                                                                                                   \
    {                                                                                                                  \
        invokeIfCallable(m_onError, (message));                                                                        \
        return;                                                                                                        \
    }                                                                                                                  \
    do                                                                                                                 \
    {                                                                                                                  \
    } while (false)

#define SIO_MESSAGE_CHECK_CONTINUE(condition, message)                                                                 \
    if ((condition))                                                                                                   \
    {                                                                                                                  \
        invokeIfCallable(m_onError, (message));                                                                        \
        continue;                                                                                                      \
    }                                                                                                                  \
    do                                                                                                                 \
    {                                                                                                                  \
    } while (false)

constexpr int SignalingProtocolVersion = 1;

SignalingClient::SignalingClient(
    SignalingServerConfiguration&& signalingServerConfiguration,
    WebrtcConfiguration&& webrtcConfiguration)
    : m_signalingServerConfiguration(move(signalingServerConfiguration)),
      m_webrtcConfiguration(move(webrtcConfiguration)),
      m_hasClosePending(false)
{
    constexpr int ReconnectAttempts = 10;
    m_sio.set_reconnect_attempts(ReconnectAttempts);

    m_internalClientThread = move(rtc::Thread::Create());
    m_internalClientThread->SetName(m_signalingServerConfiguration.clientName() + " - internal client", nullptr);
    m_internalClientThread->Start();

    m_networkThread = move(rtc::Thread::CreateWithSocketServer());
    m_networkThread->SetName(m_signalingServerConfiguration.clientName() + " - network", nullptr);
    m_networkThread->Start();
    m_workerThread = move(rtc::Thread::Create());
    m_workerThread->SetName(m_signalingServerConfiguration.clientName() + " - worker", nullptr);
    m_workerThread->Start();
    m_signalingThread = move(rtc::Thread::Create());
    m_signalingThread->SetName(m_signalingServerConfiguration.clientName() + " - signaling", nullptr);
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
        webrtc::CreateBuiltinVideoEncoderFactory(),
        webrtc::CreateBuiltinVideoDecoderFactory(),
        nullptr,  // Audio mixer,
        m_audioProcessing);

    if (!m_peerConnectionFactory)
    {
        throw runtime_error("CreatePeerConnectionFactory failed");
    }
}

/**
 * Enable or disable the TLS verification. By default, the TLS verification is
 * enabled.
 * @param isEnabled
 */
void SignalingClient::setTlsVerificationEnabled(bool isEnabled)
{
    callAsync(m_internalClientThread.get(), [this, isEnabled]() { m_sio.set_is_tls_verification_enabled(isEnabled); });
}

/**
 * @brief Connects the client the signaling server.
 */
void SignalingClient::connect()
{
    callAsync(
        m_internalClientThread.get(),
        [this]()
        {
            closeAllConnections();
            m_hasClosePending = false;
            connectSioEvents();
            m_sio.connect(m_signalingServerConfiguration.url());
        });
}

/**
 * @brief Closes all client connections (asynchronous).
 */
void SignalingClient::close()
{
    callAsync(
        m_internalClientThread.get(),
        [this]()
        {
            closeAllConnections();
            m_sio.close();
            m_hasClosePending = true;
        });
}

/**
 * @brief Closes all client connections (synchronous).
 */
void SignalingClient::closeSync()
{
    callSync(
        m_internalClientThread.get(),
        [this]()
        {
            closeAllConnections();
            m_sio.sync_close();
            m_hasClosePending = true;
        });
}

/**
 * @brief Calls all room clients.
 */
void SignalingClient::callAll()
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

            m_sio.socket()->emit("call-all");
        });
}

/**
 * @brief Calls the specified clients.
 */
void SignalingClient::callIds(const vector<string>& ids)
{
    callAsync(
        m_internalClientThread.get(),
        [this, ids]()
        {
            m_alreadyAcceptedCalls = ids;

            auto data = sio::array_message::create();
            for (const auto& id : ids)
            {
                data->get_vector().emplace_back(sio::string_message::create(id));
            }
            m_sio.socket()->emit("call-ids", data);
        });
}

/**
 * @brief Hangs up all clients.
 */
void SignalingClient::hangUpAll()
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
void SignalingClient::closeAllRoomPeerConnections()
{
    callAsync(m_internalClientThread.get(), [this]() { m_sio.socket()->emit("close-all-room-peer-connections"); });
}

/**
 * @brief Returns the connected room client ids.
 * @return The connected room client ids
 */
vector<string> SignalingClient::getConnectedRoomClientIds()
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
vector<RoomClient> SignalingClient::getRoomClients()
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

function<void(const string&, sio::message::ptr)> SignalingClient::getSendEventFunction()
{
    return [this](const string& event, sio::message::ptr message)
    { callAsync(m_internalClientThread.get(), [this, event, message]() { m_sio.socket()->emit(event, message); }); };
}

function<void(const string&)> SignalingClient::getOnErrorFunction()
{
    return [this](const string& message) { invokeIfCallable(m_onError, message); };
}

function<void(const Client&)> SignalingClient::getOnClientConnectedFunction()
{
    return [this](const Client& client) { invokeIfCallable(m_onClientConnected, client); };
}

function<void(const Client&)> SignalingClient::getOnClientDisconnectedFunction()
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

void SignalingClient::connectSioEvents()
{
    m_sio.set_open_listener([this] { onSioConnectEvent(); });
    m_sio.set_fail_listener([this] { onSioErrorEvent(); });
    m_sio.set_close_listener([this](const sio::client::close_reason& reason) { onSioDisconnectEvent(reason); });

    m_sio.on("room-clients", [this](sio::event& event) { onRoomClientsEvent(event); });

    m_sio.on("make-peer-call", [this](sio::event& event) { onMakePeerCallEvent(event); });
    m_sio.on("peer-call-received", [this](sio::event& event) { onPeerCallReceivedEvent(event); });
    m_sio.on("peer-call-answer-received", [this](sio::event& event) { onPeerCallAnswerReceivedEvent(event); });
    m_sio.on(
        "close-all-peer-connections-request-received",
        [this](sio::event& event) { onCloseAllPeerConnectionsRequestReceivedEvent(event); });

    m_sio.on("ice-candidate-received", [this](sio::event& event) { onIceCandidateReceivedEvent(event); });
}

void SignalingClient::onSioConnectEvent()
{
    auto data = sio::object_message::create();
    data->get_map()["name"] = sio::string_message::create(m_signalingServerConfiguration.clientName());
    data->get_map()["data"] = m_signalingServerConfiguration.clientData();
    data->get_map()["room"] = sio::string_message::create(m_signalingServerConfiguration.room());
    data->get_map()["password"] = sio::string_message::create(m_signalingServerConfiguration.password());
    data->get_map()["protocolVersion"] = sio::int_message::create(SignalingProtocolVersion);

    m_sio.socket()->emit("join-room", data, [this](const sio::message::list& msg) { onJoinRoomCallback(msg); });
}

void SignalingClient::onSioErrorEvent()
{
    invokeIfCallable(m_onSignalingConnectionError, "");
}

void SignalingClient::onSioDisconnectEvent(const sio::client::close_reason& reason)
{
    invokeIfCallable(m_onSignalingConnectionClosed);
}

void SignalingClient::onJoinRoomCallback(const sio::message::list& message)
{
    if (message.size() == 1 && message[0]->get_flag() == sio::message::flag_boolean)
    {
        if (message[0]->get_bool())
        {
            invokeIfCallable(m_onSignalingConnectionOpened);
        }
        else
        {
            close();
            invokeIfCallable(m_onSignalingConnectionError, "Invalid password or invalid protocol version");
        }
    }
    else
    {
        close();
        invokeIfCallable(m_onSignalingConnectionError, "Invalid join-room response");
    }
}

void SignalingClient::onRoomClientsEvent(sio::event& event)
{
    callAsync(
        m_internalClientThread.get(),
        [this, event]()
        {
            m_roomClientsById.clear();
            for (const auto& roomClient : event.get_message()->get_vector())
            {
                if (Client::isValid(roomClient))
                {
                    Client decodedClient(roomClient);
                    m_roomClientsById[decodedClient.id()] = decodedClient;
                }
            }
            invokeIfCallable(m_onRoomClientsChanged, getRoomClients());
        });
}

void SignalingClient::onMakePeerCallEvent(sio::event& event)
{
    SIO_MESSAGE_CHECK_RETURN(
        event.get_message()->get_flag() != sio::message::flag_array,
        "Invalid onMakePeerCallEvent message (global type)");

    for (const auto& idMessage : event.get_message()->get_vector())
    {
        SIO_MESSAGE_CHECK_CONTINUE(
            idMessage->get_flag() != sio::message::flag_string,
            "Invalid onMakePeerCallEvent peer id");
        makePeerCall(idMessage->get_string());
    }
}

void SignalingClient::makePeerCall(const string& id)
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

void SignalingClient::onPeerCallReceivedEvent(sio::event& event)
{
    SIO_MESSAGE_CHECK_RETURN(
        event.get_message()->get_flag() != sio::message::flag_object,
        "Invalid onPeerCallReceivedEvent message (global type)");
    auto data = event.get_message()->get_map();
    auto fromIdIt = data.find("fromId");
    auto offerIt = data.find("offer");

    SIO_MESSAGE_CHECK_RETURN(
        fromIdIt == data.end() || offerIt == data.end(),
        "Invalid onPeerCallReceivedEvent message (fromId or offer are missing)");
    SIO_MESSAGE_CHECK_RETURN(
        fromIdIt->second->get_flag() != sio::message::flag_string ||
            offerIt->second->get_flag() != sio::message::flag_object,
        "Invalid onPeerCallReceivedEvent message (fromId or offer types)");
    auto fromId = fromIdIt->second->get_string();
    auto offer = offerIt->second->get_map();
    auto sdpIt = offer.find("sdp");
    auto typeIt = offer.find("type");

    SIO_MESSAGE_CHECK_RETURN(
        sdpIt == offer.end() || typeIt == offer.end(),
        "Invalid onPeerCallReceivedEvent message (sdp or type are missing)");
    SIO_MESSAGE_CHECK_RETURN(
        sdpIt->second->get_flag() != sio::message::flag_string ||
            typeIt->second->get_flag() != sio::message::flag_string,
        "Invalid onPeerCallReceivedEvent message (sdp or type wrong types)");
    auto sdp = sdpIt->second->get_string();
    auto type = typeIt->second->get_string();

    SIO_MESSAGE_CHECK_RETURN(type != "offer", "Invalid onPeerCallReceivedEvent message (invalid offer type)");
    receivePeerCall(fromId, sdp);
}

void SignalingClient::receivePeerCall(const string& fromId, const string& sdp)
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
                auto data = sio::object_message::create();
                data->get_map()["toId"] = sio::string_message::create(fromId);
                m_sio.socket()->emit("make-peer-call-answer", data);
                return;
            }

            m_peerConnectionHandlersById[fromId] = createConnection(fromId, fromClientIt->second, false);
            m_peerConnectionHandlersById[fromId]->receivePeerCall(sdp);
        });
}

void SignalingClient::onPeerCallAnswerReceivedEvent(sio::event& event)
{
    SIO_MESSAGE_CHECK_RETURN(
        event.get_message()->get_flag() != sio::message::flag_object,
        "Invalid onPeerCallAnswerReceivedEvent message (global type)");
    auto data = event.get_message()->get_map();
    auto fromIdIt = data.find("fromId");
    auto answerIt = data.find("answer");

    SIO_MESSAGE_CHECK_RETURN(
        fromIdIt == data.end() || fromIdIt->second->get_flag() != sio::message::flag_string,
        "Invalid onPeerCallAnswerReceivedEvent message (fromId type)");
    auto fromId = fromIdIt->second->get_string();

    if (answerIt == data.end() || answerIt->second->get_flag() != sio::message::flag_object)
    {
        callAsync(
            m_internalClientThread.get(),
            [this, fromId]()
            {
                removeConnection(fromId);
                auto clientIt = m_roomClientsById.find(fromId);
                invokeIfCallable(m_onCallRejected, clientIt->second);
            });
        return;
    }
    auto answer = answerIt->second->get_map();
    auto sdpIt = answer.find("sdp");
    auto typeIt = answer.find("type");

    SIO_MESSAGE_CHECK_RETURN(
        sdpIt == answer.end() || typeIt == answer.end(),
        "Invalid onPeerCallAnswerReceivedEvent message (sdp "
        "or type are missing)");
    SIO_MESSAGE_CHECK_RETURN(
        sdpIt->second->get_flag() != sio::message::flag_string ||
            typeIt->second->get_flag() != sio::message::flag_string,
        "Invalid onPeerCallAnswerReceivedEvent message (sdp or type types)");
    auto sdp = sdpIt->second->get_string();
    auto type = typeIt->second->get_string();

    SIO_MESSAGE_CHECK_RETURN(type != "answer", "Invalid onPeerCallAnswerReceivedEvent message (invalid answer type)");
    receivePeerCallAnswer(fromId, sdp);
}

void SignalingClient::receivePeerCallAnswer(const string& fromId, const string& sdp)
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

void SignalingClient::onCloseAllPeerConnectionsRequestReceivedEvent(sio::event& event)
{
    log("onCloseAllPeerConnectionsRequestReceivedEvent");
    hangUpAll();
}

void SignalingClient::onIceCandidateReceivedEvent(sio::event& event)
{
    SIO_MESSAGE_CHECK_RETURN(
        event.get_message()->get_flag() != sio::message::flag_object,
        "Invalid onIceCandidateReceivedEvent message (global type)");
    auto data = event.get_message()->get_map();
    auto fromIdIt = data.find("fromId");
    auto candidateIt = data.find("candidate");

    SIO_MESSAGE_CHECK_RETURN(
        fromIdIt == data.end() || candidateIt == data.end(),
        "Invalid onIceCandidateReceivedEvent message "
        "(fromId or candidate are missing)");

    if (candidateIt->second->get_flag() == sio::message::flag_null)
    {
        return;
    }
    SIO_MESSAGE_CHECK_RETURN(
        fromIdIt->second->get_flag() != sio::message::flag_string ||
            candidateIt->second->get_flag() != sio::message::flag_object,
        "Invalid onIceCandidateReceivedEvent message (fromId or candidate wrong "
        "types)");
    auto fromId = fromIdIt->second->get_string();
    auto candidate = candidateIt->second->get_map();
    auto sdpMidIt = candidate.find("sdpMid");
    auto sdpMLineIndexIt = candidate.find("sdpMLineIndex");
    auto sdpIt = candidate.find("candidate");

    SIO_MESSAGE_CHECK_RETURN(
        sdpMidIt == candidate.end() || sdpMLineIndexIt == candidate.end() || sdpIt == candidate.end(),
        "Invalid onIceCandidateReceivedEvent message "
        "(sdpMid, sdpMLineIndex or candidate are missing)");
    SIO_MESSAGE_CHECK_RETURN(
        sdpMidIt->second->get_flag() != sio::message::flag_string ||
            sdpMLineIndexIt->second->get_flag() != sio::message::flag_integer ||
            sdpIt->second->get_flag() != sio::message::flag_string,
        "Invalid onIceCandidateReceivedEvent message (sdpMid, sdpMLineIndex or "
        "candidate wrong types)");
    auto sdpMid = sdpMidIt->second->get_string();
    auto sdpMLineIndex = sdpMLineIndexIt->second->get_int();
    auto sdp = sdpIt->second->get_string();

    receiveIceCandidate(fromId, sdpMid, static_cast<int>(sdpMLineIndex), sdp);
}

void SignalingClient::receiveIceCandidate(
    const string& fromId,
    const string& sdpMid,
    int sdpMLineIndex,
    const string& sdp)
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

void SignalingClient::closeAllConnections()
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

bool SignalingClient::getCallAcceptance(const string& id)
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
    SignalingClient::createConnection(const string& peerId, const Client& peerClient, bool isCaller)
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
                throw std::runtime_error(
                    std::string("Failed to create peer connection: ") + peerConnection.error().message());
            }
            return handler;
        });
}

void SignalingClient::removeConnection(const string& id)
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
