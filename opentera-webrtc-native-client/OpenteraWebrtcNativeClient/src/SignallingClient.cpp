#include <OpenteraWebrtcNativeClient/SignallingClient.h>

#include <api/create_peerconnection_factory.h>
#include <api/audio_codecs/builtin_audio_decoder_factory.h>
#include <api/audio_codecs/builtin_audio_encoder_factory.h>
#include <api/video_codecs/builtin_video_decoder_factory.h>
#include <api/video_codecs/builtin_video_encoder_factory.h>

using namespace introlab;
using namespace std;

#define SIO_MESSAGE_CHECK_RETURN(condition, message) \
        if ((condition)) \
        { \
            invokeIfCallable(m_onError, (message)); \
            return; \
        } \
        do {} while(false)

#define SIO_MESSAGE_CHECK_CONTINUE(condition, message) \
        if ((condition)) \
        { \
            invokeIfCallable(m_onError, (message)); \
            continue; \
        } \
        do {} while(false)


SignallingClient::SignallingClient(const SignallingServerConfiguration& signallingServerConfiguration,
                                   const WebrtcConfiguration& webrtcConfiguration) :
        m_signallingServerConfiguration(signallingServerConfiguration), m_webrtcConfiguration(webrtcConfiguration),
        m_hasClosePending(false)
{
    constexpr int ReconnectAttempts = 10;
    m_sio.set_reconnect_attempts(ReconnectAttempts);

    m_networkThread = move(rtc::Thread::CreateWithSocketServer());
    m_networkThread->SetName(m_signallingServerConfiguration.clientName() + " - network", nullptr);
    m_networkThread->Start();
    m_workerThread = move(rtc::Thread::Create());
    m_workerThread->SetName(m_signallingServerConfiguration.clientName() + " - worker", nullptr);
    m_workerThread->Start();
    m_signallingThread = move(rtc::Thread::Create());
    m_signallingThread->SetName(m_signallingServerConfiguration.clientName() + " - signalling", nullptr);
    m_signallingThread->Start();

    m_peerConnectionFactory = webrtc::CreatePeerConnectionFactory(m_networkThread.get(),
            m_workerThread.get(),
            m_signallingThread.get(),
            nullptr, // Default adm
            webrtc::CreateBuiltinAudioEncoderFactory(),
            webrtc::CreateBuiltinAudioDecoderFactory(),
            webrtc::CreateBuiltinVideoEncoderFactory(),
            webrtc::CreateBuiltinVideoDecoderFactory(),
            nullptr, // Audio mixer,
            nullptr); // Audio processing

    if (!m_peerConnectionFactory)
    {
        throw runtime_error("CreatePeerConnectionFactory failed");
    }
}

SignallingClient::~SignallingClient()
{
    closeAllConnections();
}

void SignallingClient::connect()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);
    m_hasClosePending = false;
    connectSioEvents();
    m_sio.connect(m_signallingServerConfiguration.url());
}

void SignallingClient::close()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);
    if (m_sio.opened())
    {
        m_sio.close();
    }
    m_hasClosePending = true;

    closeAllConnections();
}

void SignallingClient::closeSync()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);
    if (m_sio.opened())
    {
        m_sio.sync_close();
    }
    m_hasClosePending = true;

    closeAllConnections();
}

void SignallingClient::callAll()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);

    m_alreadyAcceptedCalls.clear();
    for (const auto& pair : m_roomClientsById)
    {
        m_alreadyAcceptedCalls.push_back(pair.first);
    }

    m_sio.socket()->emit("call-all");
}

void SignallingClient::callIds(const vector<string>& ids)
{
    lock_guard<recursive_mutex> lock(m_sioMutex);

    m_alreadyAcceptedCalls = ids;

    auto data = sio::array_message::create();
    for (const auto& id : ids)
    {
        data->get_vector().push_back(sio::string_message::create(id));
    }
    m_sio.socket()->emit("call-ids", data);
}

void SignallingClient::hangUpAll()
{
    closeAllConnections();
    {
        lock_guard<recursive_mutex> lock(m_sioMutex);
        invokeIfCallable(m_onRoomClientsChanged, getRoomClients());
    }
}

void SignallingClient::closeAllRoomPeerConnections()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);
    m_sio.socket()->emit("close-all-room-peer-connections");
}

vector<string> SignallingClient::getConnectedRoomClientIds()
{
    lock_guard<recursive_mutex> lock(m_peerConnectionMutex);
    vector<string> ids(m_peerConnectionsHandlerById.size());
    for (const auto& pair : m_peerConnectionsHandlerById)
    {
        ids.push_back(pair.first);
    }
    return ids;
}

vector<RoomClient> SignallingClient::getRoomClients()
{
    vector<Client> clients;
    {
        lock_guard<recursive_mutex> lock(m_sioMutex);
        for (const auto& pair : m_roomClientsById)
        {
            clients.push_back(pair.second);
        }
    }
    {
        lock_guard<recursive_mutex> lock(m_peerConnectionMutex);

        vector<RoomClient> roomClients(clients.size());
        for (const auto &client : clients)
        {
            bool isConnected = m_peerConnectionsHandlerById.find(client.id()) != m_peerConnectionsHandlerById.end() ||
                    client.id() == id();
            roomClients.emplace_back(client, isConnected);
        }

        return roomClients;
    }
}

function<void(const string&, sio::message::ptr)> SignallingClient::getSendEventFunction()
{
    return [this](const string& event, sio::message::ptr message)
    {
        lock_guard<recursive_mutex> lock(m_sioMutex);
        m_sio.socket()->emit(event, message);
    };
}

function<void(const string&)> SignallingClient::getOnErrorFunction()
{
    return [this](const string& message)
    {
        invokeIfCallable(m_onError, message);
    };
}

function<void(const Client&)> SignallingClient::getOnClientConnectedFunction()
{
    return [this](const Client& client)
    {
        invokeIfCallable(m_onClientConnected, client);
    };
}

function<void(const Client&)> SignallingClient::getOnClientDisconnectedFunction()
{
    return [this](const Client& client)
    {
        removeConnection(client.id());
        invokeIfCallable(m_onClientDisconnected, client);
    };
}

void SignallingClient::connectSioEvents()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);

    m_sio.set_open_listener([this] { onSioConnectEvent(); });
    m_sio.set_fail_listener([this] { onSioErrorEvent(); });
    m_sio.set_close_listener([this](const sio::client::close_reason& reason) { onSioDisconnectEvent(reason); });

    m_sio.socket()->on("room-clients", [this] (sio::event& event) { onRoomClientsEvent(event); });

    m_sio.socket()->on("make-peer-call", [this] (sio::event& event) { onMakePeerCallEvent(event); });
    m_sio.socket()->on("peer-call-received", [this] (sio::event& event) { onPeerCallReceivedEvent(event); });
    m_sio.socket()->on("peer-call-answer-received",
            [this] (sio::event& event) { onPeerCallAnswerReceivedEvent(event); });
    m_sio.socket()->on("close-all-peer-connections-request-received",
        [this] (sio::event& event) { onCloseAllPeerConnectionsRequestReceivedEvent(event); });

    m_sio.socket()->on("ice-candidate-received", [this] (sio::event& event) { onIceCandidateReceivedEvent(event); });
}

void SignallingClient::onSioConnectEvent()
{
    auto data = sio::object_message::create();
    data->get_map()["name"] = sio::string_message::create(m_signallingServerConfiguration.clientName());
    data->get_map()["data"] = m_signallingServerConfiguration.clientData();
    data->get_map()["room"] = sio::string_message::create(m_signallingServerConfiguration.room());
    data->get_map()["password"] = sio::string_message::create(m_signallingServerConfiguration.password());

    lock_guard<recursive_mutex> lock(m_sioMutex);
    m_sio.socket()->emit("join-room", data, [this](const sio::message::list& msg) { onJoinRoomCallback(msg); });
}

void SignallingClient::onSioErrorEvent()
{
    invokeIfCallable(m_onSignallingConnectionError, "");
}

void SignallingClient::onSioDisconnectEvent(const sio::client::close_reason& reason)
{
    invokeIfCallable(m_onSignallingConnectionClosed);
}

void SignallingClient::onJoinRoomCallback(const sio::message::list& message)
{
    if (message.size() == 1 && message[0]->get_flag() == sio::message::flag_boolean)
    {
        if (message[0]->get_bool())
        {
            invokeIfCallable(m_onSignallingConnectionOpen);
        }
        else
        {
            close();
            invokeIfCallable(m_onSignallingConnectionError, "Invalid password");
        }
    }
    else
    {
        close();
        invokeIfCallable(m_onSignallingConnectionError, "Invalid join-room response");
    }
}

void SignallingClient::onRoomClientsEvent(sio::event& event)
{
    SIO_MESSAGE_CHECK_RETURN(event.get_message()->get_flag() != sio::message::flag_array,
            "Invalid onRoomClientsEvent message (global type)");

    {
        lock_guard<recursive_mutex> lock(m_sioMutex);

        m_roomClientsById.clear();
        for (const auto& roomClient : event.get_message()->get_vector())
        {
            if (Client::isValid(roomClient))
            {
                Client decodedClient(roomClient);
                m_roomClientsById[decodedClient.id()] = decodedClient;
            }
        }
    }

    invokeIfCallable(m_onRoomClientsChanged, getRoomClients());
}

void SignallingClient::onMakePeerCallEvent(sio::event& event)
{
    SIO_MESSAGE_CHECK_RETURN(event.get_message()->get_flag() != sio::message::flag_array,
            "Invalid onMakePeerCallEvent message (global type)");

    for (const auto& idMessage : event.get_message()->get_vector())
    {
        SIO_MESSAGE_CHECK_CONTINUE(idMessage->get_flag() != sio::message::flag_string,
                "Invalid onMakePeerCallEvent peer id");
        makePeerCall(idMessage->get_string());
    }
}

void SignallingClient::makePeerCall(const string& id)
{
    Client client;
    {
        lock_guard<recursive_mutex> lock(m_sioMutex);
        auto clientIt = m_roomClientsById.find(id);
        if (clientIt == m_roomClientsById.end()) { return; }
        client = clientIt->second;
    }
    {
        lock_guard<recursive_mutex> lock(m_peerConnectionMutex);
        if (m_peerConnectionsHandlerById.find(id) != m_peerConnectionsHandlerById.end()) { return; }
        if (!getCallAcceptance(id)) { return; }

        m_peerConnectionsHandlerById[id] = createConnection(id, client, true);
        m_peerConnectionsHandlerById[id]->makePeerCall();
    }
}

void SignallingClient::onPeerCallReceivedEvent(sio::event& event)
{
    SIO_MESSAGE_CHECK_RETURN(event.get_message()->get_flag() != sio::message::flag_object,
            "Invalid onPeerCallReceivedEvent message (global type)");
    auto data = event.get_message()->get_map();
    auto fromIdIt = data.find("fromId");
    auto offerIt = data.find("offer");

    SIO_MESSAGE_CHECK_RETURN(fromIdIt == data.end() || fromIdIt->second->get_flag() != sio::message::flag_string,
            "Invalid onPeerCallReceivedEvent message (fromId type)");
    auto fromId = fromIdIt->second->get_string();

    if (offerIt == data.end() || offerIt->second->get_flag() != sio::message::flag_object)
    {
        lock_guard<recursive_mutex> lock(m_sioMutex);
        auto clientIt = m_roomClientsById.find(fromId);
        invokeIfCallable(m_onCallRejected, clientIt->second);
        return;
    }
    auto offer = offerIt->second->get_map();
    auto sdpIt = offer.find("sdp");
    auto typeIt = offer.find("type");

    SIO_MESSAGE_CHECK_RETURN(sdpIt == offer.end() || typeIt == offer.end(),
            "Invalid onPeerCallReceivedEvent message (sdp or type are missing)");
    SIO_MESSAGE_CHECK_RETURN(sdpIt->second->get_flag() != sio::message::flag_string ||
            typeIt->second->get_flag() != sio::message::flag_string,
            "Invalid onPeerCallReceivedEvent message (sdp or type wrong types)");
    auto sdp = sdpIt->second->get_string();
    auto type = typeIt->second->get_string();

    SIO_MESSAGE_CHECK_RETURN(type != "offer", "Invalid onPeerCallReceivedEvent message (invalid offer type)");
    receivePeerCall(fromId, sdp);
}

void SignallingClient::receivePeerCall(const string& fromId, const string& sdp)
{
    Client fromClient;
    {
        lock_guard<recursive_mutex> lock(m_sioMutex);
        auto fromClientIt = m_roomClientsById.find(fromId);
        if (fromClientIt == m_roomClientsById.end()) { return; }
        fromClient = fromClientIt->second;
    }
    {
        lock_guard<recursive_mutex> lock(m_peerConnectionMutex);
        if (m_peerConnectionsHandlerById.find(fromId) != m_peerConnectionsHandlerById.end()) { return; }
        if (!getCallAcceptance(fromId)) { return; }

        m_peerConnectionsHandlerById[fromId] = createConnection(fromId, fromClient, false);
        m_peerConnectionsHandlerById[fromId]->receivePeerCall(sdp);
    }
}

void SignallingClient::onPeerCallAnswerReceivedEvent(sio::event& event)
{
    SIO_MESSAGE_CHECK_RETURN(event.get_message()->get_flag() != sio::message::flag_object,
            "Invalid onPeerCallAnswerReceivedEvent message (global type)");
    auto data = event.get_message()->get_map();
    auto fromIdIt = data.find("fromId");
    auto answerIt = data.find("answer");

    SIO_MESSAGE_CHECK_RETURN(fromIdIt == data.end() || answerIt == data.end(),
            "Invalid onPeerCallAnswerReceivedEvent message (fromId or answer are missing)");
    SIO_MESSAGE_CHECK_RETURN(fromIdIt->second->get_flag() != sio::message::flag_string ||
            answerIt->second->get_flag() != sio::message::flag_object,
            "Invalid onPeerCallAnswerReceivedEvent message (fromId or answer types)");
    auto fromId = fromIdIt->second->get_string();
    auto answer = answerIt->second->get_map();
    auto sdpIt = answer.find("sdp");
    auto typeIt = answer.find("type");

    SIO_MESSAGE_CHECK_RETURN(sdpIt == answer.end() || typeIt == answer.end(),
            "Invalid onPeerCallAnswerReceivedEvent message (sdp or type are missing)");
    SIO_MESSAGE_CHECK_RETURN(sdpIt->second->get_flag() != sio::message::flag_string ||
            typeIt->second->get_flag() != sio::message::flag_string,
            "Invalid onPeerCallAnswerReceivedEvent message (sdp or type types)");
    auto sdp = sdpIt->second->get_string();
    auto type = typeIt->second->get_string();

    SIO_MESSAGE_CHECK_RETURN(type != "answer", "Invalid onPeerCallAnswerReceivedEvent message (invalid answer type)");
    receivePeerCallAnswer(fromId, sdp);
}

void SignallingClient::receivePeerCallAnswer(const string& fromId, const string& sdp)
{
    lock_guard<recursive_mutex> lock(m_peerConnectionMutex);
    auto peerConnectionsHandlerIt = m_peerConnectionsHandlerById.find(fromId);
    if (peerConnectionsHandlerIt == m_peerConnectionsHandlerById.end()) { return; }

    peerConnectionsHandlerIt->second->receivePeerCallAnswer(sdp);
}

void SignallingClient::onCloseAllPeerConnectionsRequestReceivedEvent(sio::event& event)
{
    hangUpAll();
}

void SignallingClient::onIceCandidateReceivedEvent(sio::event& event)
{
    SIO_MESSAGE_CHECK_RETURN(event.get_message()->get_flag() != sio::message::flag_object,
            "Invalid onIceCandidateReceivedEvent message (global type)");
    auto data = event.get_message()->get_map();
    auto fromIdIt = data.find("fromId");
    auto candidateIt = data.find("candidate");

    SIO_MESSAGE_CHECK_RETURN(fromIdIt == data.end() || candidateIt == data.end(),
            "Invalid onIceCandidateReceivedEvent message (fromId or candidate are missing)");
    SIO_MESSAGE_CHECK_RETURN(fromIdIt->second->get_flag() != sio::message::flag_string ||
            candidateIt->second->get_flag() != sio::message::flag_object,
            "Invalid onIceCandidateReceivedEvent message (fromId or candidate wrong types)");
    auto fromId = fromIdIt->second->get_string();
    auto candidate = candidateIt->second->get_map();
    auto sdpMidIt = candidate.find("sdpMid");
    auto sdpMLineIndexIt = candidate.find("sdpMLineIndex");
    auto sdpIt = candidate.find("candidate");

    SIO_MESSAGE_CHECK_RETURN(sdpMidIt == candidate.end() || sdpMLineIndexIt == candidate.end() ||
            sdpIt == candidate.end(),
            "Invalid onIceCandidateReceivedEvent message (sdpMid, sdpMLineIndex or candidate are missing)");
    SIO_MESSAGE_CHECK_RETURN(sdpMidIt->second->get_flag() != sio::message::flag_string ||
            sdpMLineIndexIt->second->get_flag() != sio::message::flag_integer ||
            sdpIt->second->get_flag() != sio::message::flag_string,
            "Invalid onIceCandidateReceivedEvent message (sdpMid, sdpMLineIndex or candidate wrong types)");
    auto sdpMid = sdpMidIt->second->get_string();
    auto sdpMLineIndex = sdpMLineIndexIt->second->get_int();
    auto sdp = sdpIt->second->get_string();

    receiveIceCandidate(fromId, sdpMid, static_cast<int>(sdpMLineIndex), sdp);
}

void SignallingClient::receiveIceCandidate(const string& fromId, const string& sdpMid, int sdpMLineIndex,
                                           const string& sdp)
{
    lock_guard<recursive_mutex> lockPeerConnection(m_peerConnectionMutex);
    auto peerConnectionsHandlerIt = m_peerConnectionsHandlerById.find(fromId);
    if (peerConnectionsHandlerIt == m_peerConnectionsHandlerById.end()) { return; }

    peerConnectionsHandlerIt->second->receiveIceCandidate(sdpMid, sdpMLineIndex, sdp);
}

void SignallingClient::closeAllConnections()
{
    lock_guard<recursive_mutex> lock(m_peerConnectionMutex);
    vector<string> ids(m_peerConnectionsHandlerById.size());
    for (const auto& pair : m_peerConnectionsHandlerById)
    {
        ids.push_back(pair.first);
    }

    for (const auto& id : ids)
    {
        removeConnection(id);
    }
}

bool SignallingClient::getCallAcceptance(const string& id)
{
    Client client;
    {
        lock_guard<recursive_mutex> lock(m_sioMutex);
        if (find(m_alreadyAcceptedCalls.begin(), m_alreadyAcceptedCalls.end(), id) != m_alreadyAcceptedCalls.end())
        {
            return true;
        }

        auto clientIt = m_roomClientsById.find(id);
        if (clientIt == m_roomClientsById.end())
        {
            return false;
        }
        else
        {
            client = clientIt->second;
        }
    }
    {
        lock_guard<recursive_mutex> lock(m_callbackMutex);
        return m_callAcceptor ? m_callAcceptor(client) : true;
    }
}

unique_ptr<PeerConnectionHandler> SignallingClient::createConnection(const string& peerId, const Client& peerClient,
        bool isCaller)
{
    lock_guard<recursive_mutex> lock(m_peerConnectionMutex);

    auto configuration = static_cast<webrtc::PeerConnectionInterface::RTCConfiguration>(m_webrtcConfiguration);
    unique_ptr<PeerConnectionHandler> handler = createPeerConnectionHandler(id(), peerClient, isCaller);
    auto peerConnection = m_peerConnectionFactory->CreatePeerConnection(configuration,
            webrtc::PeerConnectionDependencies(handler.get()));
    handler->setPeerConnection(peerConnection);
    return handler;
}

void SignallingClient::removeConnection(const string& id)
{
    {
        lock_guard<recursive_mutex> lockSio(m_sioMutex);
        auto it = find(m_alreadyAcceptedCalls.begin(), m_alreadyAcceptedCalls.end(), id);
        if (it != m_alreadyAcceptedCalls.end())
        {
            m_alreadyAcceptedCalls.erase(it);
        }
    }

    lock_guard<recursive_mutex> lock(m_peerConnectionMutex);

    auto peerConnectionIt = m_peerConnectionsHandlerById.find(id);
    if (peerConnectionIt != m_peerConnectionsHandlerById.end())
    {
        m_peerConnectionsHandlerById.erase(peerConnectionIt);
    }
}
