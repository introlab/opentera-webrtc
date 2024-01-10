#include <OpenteraWebrtcNativeClient/Signaling/WebSocketSignalingClient.h>

#include <ixwebsocket/IXNetSystem.h>

#include <mutex>

using namespace opentera;
using namespace std;

constexpr int SignalingProtocolVersion = 2;

#define JSON_CHECK_RETURN(condition, message)                                                                          \
    if ((condition))                                                                                                   \
    {                                                                                                                  \
        invokeIfCallable(m_onError, (message));                                                                        \
        return;                                                                                                        \
    }                                                                                                                  \
    do                                                                                                                 \
    {                                                                                                                  \
    } while (false)

#define JSON_CHECK_CONTINUE(condition, message)                                                                        \
    if ((condition))                                                                                                   \
    {                                                                                                                  \
        invokeIfCallable(m_onError, (message));                                                                        \
        continue;                                                                                                      \
    }                                                                                                                  \
    do                                                                                                                 \
    {                                                                                                                  \
    } while (false)

string eventToMessage(const char* event)
{
    nlohmann::json message{{"event", event}};

    return message.dump();
}

string eventToMessage(const char* event, const nlohmann::json& data)
{
    nlohmann::json message{{"event", event}, {"data", data}};

    return message.dump();
}


once_flag initNetSystemOnceFlag;

WebSocketSignalingClient::WebSocketSignalingClient(SignalingServerConfiguration configuration)
    : SignalingClient(move(configuration))
{
    constexpr int PingIntervalSecs = 10;
    m_ws.setPingInterval(PingIntervalSecs);

    call_once(initNetSystemOnceFlag, []() { ix::initNetSystem(); });
}

WebSocketSignalingClient::~WebSocketSignalingClient()
{
    m_ws.stop();
}

void WebSocketSignalingClient::setTlsVerificationEnabled(bool isEnabled)
{
    // TODO
    ix::SocketTLSOptions options;
    if (isEnabled)
    {
        options.disable_hostname_validation = false;
        options.caFile = "SYSTEM";
    }
    else
    {
        options.disable_hostname_validation = true;
        options.caFile = "NONE";
    }
    m_ws.setTLSOptions(options);
}

bool WebSocketSignalingClient::isConnected()
{
    return !m_sessionId.empty();
}

string WebSocketSignalingClient::sessionId()
{
    return m_sessionId;
}

void WebSocketSignalingClient::connect()
{
    m_sessionId = "";
    m_ws.stop();
    m_ws.setUrl(m_configuration.url());
    connectWsEvents();
    m_ws.start();
}

void WebSocketSignalingClient::close()
{
    m_ws.close();
    m_sessionId = "";
}

void WebSocketSignalingClient::closeSync()
{
    m_ws.stop();
    m_sessionId = "";
}

void WebSocketSignalingClient::callAll()
{
    m_ws.send(eventToMessage("call-all"));
}

void WebSocketSignalingClient::callIds(const vector<string>& ids)
{
    m_ws.send(eventToMessage("call-ids", ids));
}

void WebSocketSignalingClient::closeAllRoomPeerConnections()
{
    m_ws.send(eventToMessage("close-all-room-peer-connections"));
}

void WebSocketSignalingClient::callPeer(const string& toId, const string& sdp)
{
    nlohmann::json offer{{"sdp", sdp}, {"type", "offer"}};
    nlohmann::json data{{"toId", toId}, {"offer", offer}};
    m_ws.send(eventToMessage("call-peer", data));
}

void WebSocketSignalingClient::makePeerCallAnswer(const string& toId, const string& sdp)
{
    nlohmann::json offer{
        {"sdp", sdp},
        {"type", "answer"},
    };
    nlohmann::json data{{"toId", toId}, {"answer", offer}};
    m_ws.send(eventToMessage("make-peer-call-answer", data));
}

void WebSocketSignalingClient::rejectCall(const string& toId)
{
    nlohmann::json data{{"toId", toId}};
    m_ws.send(eventToMessage("make-peer-call-answer", data));
}

void WebSocketSignalingClient::sendIceCandidate(
    const string& sdpMid,
    int sdpMLineIndex,
    const string& candidate,
    const string& toId)
{
    nlohmann::json candidateJson{
        {"sdpMid", sdpMid},
        {"sdpMLineIndex", sdpMLineIndex},
        {"candidate", candidate},
    };
    nlohmann::json data{{"toId", toId}, {"candidate", candidateJson}};
    m_ws.send(eventToMessage("send-ice-candidate", data));
}

void WebSocketSignalingClient::connectWsEvents()
{
    m_ws.setOnMessageCallback(
        [this](const ix::WebSocketMessagePtr& msg)
        {
            switch (msg->type)
            {
                case ix::WebSocketMessageType::Open:
                    onWsOpenEvent();
                    break;
                case ix::WebSocketMessageType::Close:
                    onWsCloseEvent();
                    break;
                case ix::WebSocketMessageType::Error:
                    onWsErrorEvent(msg->errorInfo.reason);
                    break;
                case ix::WebSocketMessageType::Message:
                    onWsMessage(msg->str);
                    break;
                default:
                    break;
            }
        });
}

void WebSocketSignalingClient::onWsOpenEvent()
{
    nlohmann::json data{
        {"name", m_configuration.clientName()},
        {"data", m_configuration.clientData()},
        {"room", m_configuration.room()},
        {"password", m_configuration.password()},
        {"protocolVersion", SignalingProtocolVersion}};
    m_ws.send(eventToMessage("join-room", data));
}

void WebSocketSignalingClient::onWsCloseEvent()
{
    invokeIfCallable(m_onSignalingConnectionClosed);
}

void WebSocketSignalingClient::onWsErrorEvent(const string& error)
{
    invokeIfCallable(m_onSignalingConnectionError, error);
}

void WebSocketSignalingClient::onWsMessage(const string& message)
{
    nlohmann::json parsedMesssage = nlohmann::json::parse(message, nullptr, false);

    if (parsedMesssage.is_discarded() || !parsedMesssage.is_object() || !parsedMesssage.contains("event"))
    {
        return;
    }

    string event = parsedMesssage["event"];
    nlohmann::json data;
    if (parsedMesssage.contains("data"))
    {
        data = parsedMesssage["data"];
    }

    if (event == "join-room-answer")
    {
        onJoinRoomAnswerEvent(data);
    }
    else if (event == "room-clients")
    {
        onRoomClientsEvent(data);
    }
    else if (event == "make-peer-call")
    {
        onMakePeerCallEvent(data);
    }
    else if (event == "peer-call-received")
    {
        onPeerCallReceivedEvent(data);
    }
    else if (event == "peer-call-answer-received")
    {
        onPeerCallAnswerReceivedEvent(data);
    }
    else if (event == "close-all-peer-connections-request-received")
    {
        onCloseAllPeerConnectionsRequestReceivedEvent();
    }
    else if (event == "ice-candidate-received")
    {
        onIceCandidateReceivedEvent(data);
    }
}

void WebSocketSignalingClient::onJoinRoomAnswerEvent(const nlohmann::json& data)
{
    if (data.is_string())
    {
        if (data != nlohmann::json(""))
        {
            m_sessionId = data;
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

void WebSocketSignalingClient::onRoomClientsEvent(const nlohmann::json& data)
{
    vector<Client> clients;
    if (!data.is_array())
    {
        invokeIfCallable(m_onError, "Invalid room clients data");
        invokeIfCallable(m_onRoomClientsChanged, clients);
        return;
    }

    for (const auto& roomClient : data)
    {
        if (Client::isValid(roomClient))
        {
            clients.emplace_back(roomClient);
        }
    }
    invokeIfCallable(m_onRoomClientsChanged, clients);
}

void WebSocketSignalingClient::onMakePeerCallEvent(const nlohmann::json& data)
{
    JSON_CHECK_RETURN(!data.is_array(), "Invalid onMakePeerCallEvent data (global type)");

    for (const auto& id : data)
    {
        JSON_CHECK_CONTINUE(!id.is_string(), "Invalid onMakePeerCallEvent peer id");
        invokeIfCallable(m_makePeerCall, id);
    }
}

void WebSocketSignalingClient::onPeerCallReceivedEvent(const nlohmann::json& data)
{
    JSON_CHECK_RETURN(!data.is_object(), "Invalid onPeerCallReceivedEvent data (global type)");

    JSON_CHECK_RETURN(
        !data.contains("fromId") || !data.contains("offer"),
        "Invalid onPeerCallReceivedEvent data (fromId or offer are missing)");

    auto fromId = data["fromId"];
    auto offer = data["offer"];
    JSON_CHECK_RETURN(
        !fromId.is_string() || !offer.is_object(),
        "Invalid onPeerCallReceivedEvent data (fromId or offer types)");

    JSON_CHECK_RETURN(
        !offer.contains("sdp") || !offer.contains("type"),
        "Invalid onPeerCallReceivedEvent message (sdp or type are missing)");

    auto sdp = offer["sdp"];
    auto type = offer["type"];
    JSON_CHECK_RETURN(
        !sdp.is_string() || !type.is_string(),
        "Invalid onPeerCallReceivedEvent message (sdp or type wrong types)");

    JSON_CHECK_RETURN(type != "offer", "Invalid onPeerCallReceivedEvent message (invalid offer type)");
    invokeIfCallable(m_receivePeerCall, fromId, sdp);
}

void WebSocketSignalingClient::onPeerCallAnswerReceivedEvent(const nlohmann::json& data)
{
    JSON_CHECK_RETURN(!data.is_object(), "Invalid onPeerCallAnswerReceivedEvent message (global type)");

    auto fromIdIt = data.find("fromId");
    auto answerIt = data.find("answer");

    JSON_CHECK_RETURN(
        fromIdIt == data.end() || !fromIdIt->is_string(),
        "Invalid onPeerCallAnswerReceivedEvent message (fromId type)");
    auto fromId = *fromIdIt;

    if (answerIt == data.end() || !answerIt->is_object())
    {
        invokeIfCallable(m_onCallRejected, fromId);
        return;
    }

    auto answer = *answerIt;
    JSON_CHECK_RETURN(
        !answer.contains("sdp") || !answer.contains("type"),
        "Invalid onPeerCallAnswerReceivedEvent message (sdp or type are missing)");

    auto sdp = answer["sdp"];
    auto type = answer["type"];
    JSON_CHECK_RETURN(
        !sdp.is_string() || !type.is_string(),
        "Invalid onPeerCallAnswerReceivedEvent message (sdp or type types)");

    JSON_CHECK_RETURN(type != "answer", "Invalid onPeerCallAnswerReceivedEvent message (invalid answer type)");
    invokeIfCallable(m_receivePeerCallAnswer, fromId, sdp);
}

void WebSocketSignalingClient::onCloseAllPeerConnectionsRequestReceivedEvent()
{
    invokeIfCallable(m_closeAllPeerConnections);
}

void WebSocketSignalingClient::onIceCandidateReceivedEvent(const nlohmann::json& data)
{
    JSON_CHECK_RETURN(!data.is_object(), "Invalid onIceCandidateReceivedEvent message (global type)");
    JSON_CHECK_RETURN(
        !data.contains("fromId") || !data.contains("candidate"),
        "Invalid onIceCandidateReceivedEvent message (fromId or candidate are missing)");

    auto fromId = data["fromId"];
    auto candidate = data["candidate"];
    if (candidate.is_null())
    {
        return;
    }
    JSON_CHECK_RETURN(
        !fromId.is_string() || !candidate.is_object(),
        "Invalid onIceCandidateReceivedEvent message (fromId or candidate wrong types)");
    JSON_CHECK_RETURN(
        !candidate.contains("sdpMid") || !candidate.contains("sdpMLineIndex") || !candidate.contains("candidate"),
        "Invalid onIceCandidateReceivedEvent message (sdpMid, sdpMLineIndex or candidate are missing)");

    auto sdpMid = candidate["sdpMid"];
    auto sdpMLineIndex = candidate["sdpMLineIndex"];
    auto sdp = candidate["candidate"];
    JSON_CHECK_RETURN(
        !sdpMid.is_string() || !sdpMLineIndex.is_number_integer() || !sdp.is_string(),
        "Invalid onIceCandidateReceivedEvent message (sdpMid, sdpMLineIndex or candidate wrong types)");

    invokeIfCallable(m_receiveIceCandidate, fromId, sdpMid, static_cast<int>(sdpMLineIndex), sdp);
}
