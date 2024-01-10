#include <OpenteraWebrtcNativeClient/Utils/Client.h>

using namespace opentera;
using namespace std;

/**
 * @brief Creates a client with the specified values.
 *
 * @param id The client id
 * @param name The client name
 * @param data The client data
 */
Client::Client(string id, string name, nlohmann::json data) : m_id(move(id)), m_name(move(name)), m_data(move(data)) {}

Client::Client(const nlohmann::json& message)
{
    if (isValid(message))
    {
        m_id = message["id"];
        m_name = message["name"];
        m_data = message["data"];
    }
}

bool Client::isValid(const nlohmann::json& message)
{
    if (!message.is_object())
    {
        return false;
    }
    if (!message.contains("id") || !message.contains("name") || !message.contains("data"))
    {
        return false;
    }

    return message["id"].is_string() && message["name"].is_string();
}

/**
 * @brief Creates a room client with the default values.
 */
RoomClient::RoomClient() : m_isConnected(false) {}

/**
 * @brief Creates a room client with the specified values.
 *
 * @param id The client id
 * @param name The client name
 * @param data The client data
 * @param isConnected Indicates if the client is connected (RTCPeerConnection)
 */
RoomClient::RoomClient(string id, string name, nlohmann::json data, bool isConnected)
    : m_id(move(id)),
      m_name(move(name)),
      m_data(move(data)),
      m_isConnected(isConnected)
{
}

/**
 * @brief Creates a room client from a client.
 *
 * @param client The client
 * @param isConnected Indicates if the client is connected (RTCPeerConnection)
 */
RoomClient::RoomClient(const Client& client, bool isConnected)
    : m_id(client.id()),
      m_name(client.name()),
      m_data(client.data()),
      m_isConnected(isConnected)
{
}
