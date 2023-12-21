#include <OpenteraWebrtcNativeClient/Signaling/SioSignalingClient.h>

using namespace opentera;
using namespace std;

constexpr int SignalingProtocolVersion = 1;

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

SioSignalingClient::SioSignalingClient(SignalingServerConfiguration configuration)
    : SignalingClient(move(configuration)),
      m_hasClosePending(false)
{
    constexpr int ReconnectAttempts = 10;
    m_sio.set_reconnect_attempts(ReconnectAttempts);
}

SioSignalingClient::~SioSignalingClient()
{
    closeSync();
}

void SioSignalingClient::setTlsVerificationEnabled(bool isEnabled)
{
    m_sio.set_is_tls_verification_enabled(isEnabled);
}

bool SioSignalingClient::isConnected()
{
    return m_sio.opened();
}

string SioSignalingClient::sessionId()
{
    return m_hasClosePending ? "" : m_sio.get_sessionid();
}

void SioSignalingClient::connect()
{
    m_hasClosePending = false;
    connectSioEvents();
    m_sio.connect(m_configuration.url());
}

void SioSignalingClient::close()
{
    m_sio.close();
    m_hasClosePending = true;
}

void SioSignalingClient::closeSync()
{
    m_sio.sync_close();
    m_hasClosePending = true;
}

void SioSignalingClient::callAll()
{
    m_sio.socket()->emit("call-all");
}

void SioSignalingClient::callIds(const vector<string>& ids)
{
    auto data = sio::array_message::create();
    for (const auto& id : ids)
    {
        data->get_vector().emplace_back(sio::string_message::create(id));
    }
    m_sio.socket()->emit("call-ids", data);
}

void SioSignalingClient::closeAllRoomPeerConnections()
{
    m_sio.socket()->emit("close-all-room-peer-connections");
}

void SioSignalingClient::callPeer(const string& toId, const string& sdp)
{
    auto offer = sio::object_message::create();
    offer->get_map()["sdp"] = sio::string_message::create(sdp);

    auto data = sio::object_message::create();
    data->get_map()["toId"] = sio::string_message::create(toId);

    offer->get_map()["type"] = sio::string_message::create("offer");
    data->get_map()["offer"] = offer;

    m_sio.socket()->emit("call-peer", data);
}

void SioSignalingClient::makePeerCallAnswer(const string& toId, const string& sdp)
{
    auto offer = sio::object_message::create();
    offer->get_map()["sdp"] = sio::string_message::create(sdp);

    auto data = sio::object_message::create();
    data->get_map()["toId"] = sio::string_message::create(toId);

    offer->get_map()["type"] = sio::string_message::create("answer");
    data->get_map()["answer"] = offer;
    m_sio.socket()->emit("make-peer-call-answer", data);
}

void SioSignalingClient::rejectCall(const string& toId)
{
    auto data = sio::object_message::create();
    data->get_map()["toId"] = sio::string_message::create(toId);
    m_sio.socket()->emit("make-peer-call-answer", data);
}

void SioSignalingClient::sendIceCandidate(
    const string& sdpMid,
    int sdpMLineIndex,
    const string& candidate,
    const string& toId)
{
    auto candidateMessage = sio::object_message::create();
    candidateMessage->get_map()["sdpMid"] = sio::string_message::create(sdpMid);
    candidateMessage->get_map()["sdpMLineIndex"] = sio::int_message::create(sdpMLineIndex);
    candidateMessage->get_map()["candidate"] = sio::string_message::create(candidate);

    auto data = sio::object_message::create();
    data->get_map()["toId"] = sio::string_message::create(toId);
    data->get_map()["candidate"] = candidateMessage;

    m_sio.socket()->emit("send-ice-candidate", data);
}

void SioSignalingClient::connectSioEvents()
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

void SioSignalingClient::onSioConnectEvent()
{
    auto data = sio::object_message::create();
    data->get_map()["name"] = sio::string_message::create(m_configuration.clientName());
    data->get_map()["data"] = m_configuration.clientData();
    data->get_map()["room"] = sio::string_message::create(m_configuration.room());
    data->get_map()["password"] = sio::string_message::create(m_configuration.password());
    data->get_map()["protocolVersion"] = sio::int_message::create(SignalingProtocolVersion);

    m_sio.socket()->emit("join-room", data, [this](const sio::message::list& msg) { onJoinRoomCallback(msg); });
}

void SioSignalingClient::onSioErrorEvent()
{
    invokeIfCallable(m_onSignalingConnectionError, "");
}

void SioSignalingClient::onSioDisconnectEvent(const sio::client::close_reason& reason)
{
    invokeIfCallable(m_onSignalingConnectionClosed);
}

void SioSignalingClient::onJoinRoomCallback(const sio::message::list& message)
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

void SioSignalingClient::onRoomClientsEvent(sio::event& event)
{
    vector<Client> clients;
    for (const auto& roomClient : event.get_message()->get_vector())
    {
        if (Client::isValid(roomClient))
        {
            clients.emplace_back(roomClient);
        }
    }
    invokeIfCallable(m_onRoomClientsChanged, clients);
}

void SioSignalingClient::onMakePeerCallEvent(sio::event& event)
{
    SIO_MESSAGE_CHECK_RETURN(
        event.get_message()->get_flag() != sio::message::flag_array,
        "Invalid onMakePeerCallEvent message (global type)");

    for (const auto& idMessage : event.get_message()->get_vector())
    {
        SIO_MESSAGE_CHECK_CONTINUE(
            idMessage->get_flag() != sio::message::flag_string,
            "Invalid onMakePeerCallEvent peer id");
        invokeIfCallable(m_makePeerCall, idMessage->get_string());
    }
}

void SioSignalingClient::onPeerCallReceivedEvent(sio::event& event)
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
    invokeIfCallable(m_receivePeerCall, fromId, sdp);
}

void SioSignalingClient::onPeerCallAnswerReceivedEvent(sio::event& event)
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
        invokeIfCallable(m_onCallRejected, fromId);
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
    invokeIfCallable(m_receivePeerCallAnswer, fromId, sdp);
}

void SioSignalingClient::onCloseAllPeerConnectionsRequestReceivedEvent(sio::event& event)
{
    invokeIfCallable(m_closeAllPeerConnections);
}

void SioSignalingClient::onIceCandidateReceivedEvent(sio::event& event)
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

    invokeIfCallable(m_receiveIceCandidate, fromId, sdpMid, static_cast<int>(sdpMLineIndex), sdp);
}
