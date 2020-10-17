#include <OpenteraWebrtcNativeClient/SignallingClient.h>

#include <api/create_peerconnection_factory.h>
#include <api/audio_codecs/builtin_audio_decoder_factory.h>
#include <api/audio_codecs/builtin_audio_encoder_factory.h>
#include <api/video_codecs/builtin_video_decoder_factory.h>
#include <api/video_codecs/builtin_video_encoder_factory.h>

using namespace introlab;
using namespace std;

SignalingClient::SignalingClient(const string& url, const string& clientName, const sio::message::ptr& clientData,
        const string& room, const string& password, const string& iceServers) :
        m_url(url), m_clientName(clientName), m_clientData(clientData), m_room(room), m_password(password),
        m_iceServers(iceServers), m_hasClosePending(false)
{
    constexpr int ReconnectAttempts = 10;
    m_sio.set_reconnect_attempts(ReconnectAttempts);
    connectSioEvents();

    m_peerConnectionFactoryInterface = webrtc::CreatePeerConnectionFactory(nullptr, // Network thread
            nullptr, // Worker thread
            nullptr, // Signaling thread
            nullptr, // Default adm
            webrtc::CreateBuiltinAudioEncoderFactory(),
            webrtc::CreateBuiltinAudioDecoderFactory(),
            webrtc::CreateBuiltinVideoEncoderFactory(),
            webrtc::CreateBuiltinVideoDecoderFactory(),
            nullptr, // Audio mixer,
            nullptr); // Audio processing

    if (!m_peerConnectionFactoryInterface)
    {
        throw runtime_error("Failed to create PeerConnectionFactory");
    }
}

SignalingClient::~SignalingClient()
{
}

void SignalingClient::connect()
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
    m_hasClosePending = false;
    m_sio.connect(m_url);
}

void SignalingClient::close()
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
    if (m_sio.opened())
    {
        m_sio.close();
    }
    m_hasClosePending = true;

    // TODO close all peerConnections
}

void SignalingClient::closeSync()
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
    if (m_sio.opened())
    {
        m_sio.sync_close();
    }
    m_hasClosePending = true;

    // TODO close all peerConnections
}

void SignalingClient::callAll()
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);

    m_alreadyAcceptedCalls.clear();
    for (const auto& pair : m_roomClientsById)
    {
        m_alreadyAcceptedCalls.push_back(pair.first);
    }

    m_sio.socket()->emit("call-all");
}

void SignalingClient::callIds(const vector<string>& ids)
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);

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
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
    throw runtime_error("Not implemented");
}

void SignalingClient::closeAllRoomPeerConnections()
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
    m_sio.socket()->emit("close-all-room-peer-connections");
}

vector<string> SignalingClient::getConnectedRoomClientIds()
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
    return {};
}

vector<RoomClient> SignalingClient::getRoomClients()
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);

    vector<RoomClient> roomClients(m_roomClientsById.size());
    for (const auto& pair : m_roomClientsById)
    {
        bool isConnected = m_peerConnectionsById.find(pair.first) != m_peerConnectionsById.end() || pair.first == id();
        roomClients.emplace_back(pair.second, isConnected);
    }

    return roomClients;
}

void SignalingClient::connectSioEvents()
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);

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

    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
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
        std::lock_guard<std::recursive_mutex> lock(m_sioMutex);

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
