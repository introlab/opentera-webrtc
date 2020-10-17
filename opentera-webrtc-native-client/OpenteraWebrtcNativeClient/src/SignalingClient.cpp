#include <OpenteraWebrtcNativeClient/SignallingClient.h>

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
    throw runtime_error("Not implemented");
}

void SignalingClient::callIds(const vector<string>& ids)
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
    throw runtime_error("Not implemented");
}

void SignalingClient::hangUpAll()
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
    throw runtime_error("Not implemented");
}

void SignalingClient::closeAllRoomPeerConnections()
{
    std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
    throw runtime_error("Not implemented");
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
