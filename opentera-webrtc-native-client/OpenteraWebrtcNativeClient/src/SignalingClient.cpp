#include <OpenteraWebrtcNativeClient/SignallingClient.h>

#include <api/create_peerconnection_factory.h>
#include <api/audio_codecs/builtin_audio_decoder_factory.h>
#include <api/audio_codecs/builtin_audio_encoder_factory.h>
#include <api/video_codecs/builtin_video_decoder_factory.h>
#include <api/video_codecs/builtin_video_encoder_factory.h>

using namespace introlab;
using namespace std;

SignalingClient::SignalingClient(const string& url, const string& clientName, const sio::message::ptr& clientData,
        const string& room, const string& password, const vector<IceServer>& iceServers) :
        m_url(url), m_clientName(clientName), m_clientData(clientData), m_room(room), m_password(password),
        m_iceServers(iceServers), m_hasClosePending(false)
{
    constexpr int ReconnectAttempts = 10;
    m_sio.set_reconnect_attempts(ReconnectAttempts);

    m_peerConnectionFactory = webrtc::CreatePeerConnectionFactory(nullptr, // Network thread
            nullptr, // Worker thread
            nullptr, // Signaling thread
            nullptr, // Default adm
            webrtc::CreateBuiltinAudioEncoderFactory(),
            webrtc::CreateBuiltinAudioDecoderFactory(),
            webrtc::CreateBuiltinVideoEncoderFactory(),
            webrtc::CreateBuiltinVideoDecoderFactory(),
            nullptr, // Audio mixer,
            nullptr); // Audio processing

    if (!m_peerConnectionFactory)
    {
        throw runtime_error("Failed to create PeerConnectionFactory");
    }
}

SignalingClient::~SignalingClient()
{
}

void SignalingClient::connect()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);
    m_hasClosePending = false;
    connectSioEvents();
    m_sio.connect(m_url);
}

void SignalingClient::close()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);
    if (m_sio.opened())
    {
        m_sio.close();
    }
    m_hasClosePending = true;

    closeAllConnections();
}

void SignalingClient::closeSync()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);
    if (m_sio.opened())
    {
        m_sio.sync_close();
    }
    m_hasClosePending = true;

    closeAllConnections();
}

void SignalingClient::callAll()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);

    m_alreadyAcceptedCalls.clear();
    for (const auto& pair : m_roomClientsById)
    {
        m_alreadyAcceptedCalls.push_back(pair.first);
    }

    m_sio.socket()->emit("call-all");
}

void SignalingClient::callIds(const vector<string>& ids)
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

void SignalingClient::hangUpAll()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);
    closeAllConnections();
    invokeIfCallable(m_onRoomClientsChanged, getRoomClients());
}

void SignalingClient::closeAllRoomPeerConnections()
{
    lock_guard<recursive_mutex> lock(m_sioMutex);
    m_sio.socket()->emit("close-all-room-peer-connections");
}

vector<string> SignalingClient::getConnectedRoomClientIds()
{
    lock_guard<recursive_mutex> lockPeerConnection(m_peerConnectionMutex);
    vector<string> ids(m_peerConnectionsHandlerById.size());
    for (const auto& pair : m_peerConnectionsHandlerById)
    {
        ids.push_back(pair.first);
    }
    return ids;
}

vector<RoomClient> SignalingClient::getRoomClients()
{
    lock_guard<recursive_mutex> lockSio(m_sioMutex);
    lock_guard<recursive_mutex> lockPeerConnection(m_peerConnectionMutex);

    vector<RoomClient> roomClients(m_roomClientsById.size());
    for (const auto& pair : m_roomClientsById)
    {
        bool isConnected = m_peerConnectionsHandlerById.find(pair.first) != m_peerConnectionsHandlerById.end() ||
                pair.first == id();
        roomClients.emplace_back(pair.second, isConnected);
    }

    return roomClients;
}

function<void(const string&, sio::message::ptr)> SignalingClient::getSendEventFunction()
{
    return [this](const string& event, sio::message::ptr message)
    {
        lock_guard<recursive_mutex> lock(m_sioMutex);
        m_sio.socket()->emit(event, message);
    };
}

function<void(const string&)> SignalingClient::getOnErrorFunction()
{
    return [this](const string& message)
    {
        invokeIfCallable(m_onError, message);
    };
}

void SignalingClient::connectSioEvents()
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

void SignalingClient::onSioConnectEvent()
{
    auto data = sio::object_message::create();
    data->get_map()["name"] = sio::string_message::create(m_clientName);
    data->get_map()["data"] = m_clientData;
    data->get_map()["room"] = sio::string_message::create(m_room);
    data->get_map()["password"] = sio::string_message::create(m_password);

    lock_guard<recursive_mutex> lock(m_sioMutex);
    m_sio.socket()->emit("join-room", data, [this](const sio::message::list& msg) { onJoinRoomCallback(msg); });
}

void SignalingClient::onSioErrorEvent()
{
    invokeIfCallable(m_onSignallingConnectionError, "");
}

void SignalingClient::onSioDisconnectEvent(const sio::client::close_reason& reason)
{
    invokeIfCallable(m_onSignallingConnectionClosed);
}

void SignalingClient::onJoinRoomCallback(const sio::message::list& message)
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

void SignalingClient::onRoomClientsEvent(sio::event& event)
{
    if (event.get_message()->get_flag() != sio::message::flag_array) { return; }

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

void SignalingClient::onMakePeerCallEvent(sio::event& event)
{
    if (event.get_message()->get_flag() != sio::message::flag_array) { return; }

    for (const auto& idMessage : event.get_message()->get_vector())
    {
        if (idMessage->get_flag() != sio::message::flag_string) { continue; }
        makePeerCall(idMessage->get_string());
    }
}

void SignalingClient::makePeerCall(const string& id)
{
    lock_guard<recursive_mutex> lock(m_sioMutex);
    lock_guard<recursive_mutex> lockPeerConnection(m_peerConnectionMutex);
    if (m_peerConnectionsHandlerById.find(id) != m_peerConnectionsHandlerById.end()) { return; }
    if (!getCallAcceptance(id)) { return; }

    m_peerConnectionsHandlerById[id] = createConnection(id, true);
    m_peerConnectionsHandlerById[id]->makePeerCall();
}

void SignalingClient::onPeerCallReceivedEvent(sio::event& event)
{
}

void SignalingClient::onPeerCallAnswerReceivedEvent(sio::event& event)
{
}

void SignalingClient::onCloseAllPeerConnectionsRequestReceivedEvent(sio::event& event)
{
    hangUpAll();
}

void SignalingClient::onIceCandidateReceivedEvent(sio::event& event)
{
}

void SignalingClient::closeAllConnections()
{
    lock_guard<recursive_mutex> lock(m_peerConnectionMutex);
    for (const auto& pair : m_peerConnectionsHandlerById)
    {
        removeConnection(pair.first);
    }
}

bool SignalingClient::getCallAcceptance(const string& id)
{
    lock_guard<recursive_mutex> lockSio(m_sioMutex);
    if (find(m_alreadyAcceptedCalls.begin(), m_alreadyAcceptedCalls.end(), id) != m_alreadyAcceptedCalls.end())
    {
        return true;
    }

    auto clientIt = m_roomClientsById.find(id);
    if (clientIt == m_roomClientsById.end())
    {
        return false;
    }

    lock_guard<recursive_mutex> lockCallback(m_callbackMutex);
    return m_callAcceptor ? m_callAcceptor(clientIt->second) : true;
}

unique_ptr<PeerConnectionHandler> SignalingClient::createConnection(const string& peerId, bool isCaller)
{
    lock_guard<recursive_mutex> lockSio(m_sioMutex);
    lock_guard<recursive_mutex> lock(m_peerConnectionMutex);

    webrtc::PeerConnectionInterface::RTCConfiguration configuration;
    for (const auto& iceServer : m_iceServers)
    {
        configuration.servers.push_back(static_cast<webrtc::PeerConnectionInterface::IceServer>(iceServer));
    }

    unique_ptr<PeerConnectionHandler> handler = createPeerConnectionHandler(id(), m_roomClientsById[peerId], isCaller);
    auto peerConnection = m_peerConnectionFactory->CreatePeerConnection(configuration,
            webrtc::PeerConnectionDependencies(handler.get()));
    handler->setPeerConnection(peerConnection);
    return handler;
}

void SignalingClient::removeConnection(const string& id)
{
    lock_guard<recursive_mutex> lock(m_peerConnectionMutex);

    auto peerConnectionIt = m_peerConnectionsHandlerById.find(id);
    if (peerConnectionIt != m_peerConnectionsHandlerById.end())
    {
        m_peerConnectionsHandlerById.erase(peerConnectionIt);
    }
}
