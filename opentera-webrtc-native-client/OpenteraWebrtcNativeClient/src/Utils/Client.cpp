#include <OpenteraWebrtcNativeClient/Utils/Client.h>

using namespace opentera;
using namespace std;

bool opentera::operator==(const sio::message& m1, const sio::message& m2)
{
    if (m1.get_flag() != m2.get_flag())
    {
        return false;
    }

    switch (m1.get_flag())
    {
        case sio::message::flag_integer:
            return m1.get_int() == m2.get_int();

        case sio::message::flag_double:
            return m1.get_double() == m2.get_double();

        case sio::message::flag_string:
            return m1.get_string() == m2.get_string();

        case sio::message::flag_binary:
            if (m1.get_binary() == m2.get_binary())
            {
                return true;
            }
            if (m1.get_binary() && m2.get_binary())
            {
                return *m1.get_binary() == *m2.get_binary();
            }
            else
            {
                return false;
            }

        case sio::message::flag_boolean:
            return m1.get_bool() == m2.get_bool();

        case sio::message::flag_null:
            return true;

        case sio::message::flag_array:
            if (m1.get_vector().size() != m2.get_vector().size())
            {
                return false;
            }
            for (size_t i = 0; i < m1.get_vector().size(); i++)
            {
                if (*m1.get_vector()[i] != *m2.get_vector()[i])
                {
                    return false;
                }
            }
            return true;

        case sio::message::flag_object:
            for (const auto& m1Pair : m1.get_map())
            {
                if (m2.get_map().find(m1Pair.first) == m2.get_map().end() ||
                    *m1Pair.second != *m2.get_map().at(m1Pair.first))
                {
                    return false;
                }
            }
            return true;
    }
    return true;
}

bool opentera::operator!=(const sio::message& m1, const sio::message& m2)
{
    return !(m1 == m2);
}

/**
 * @brief Creates a client with the specified values.
 *
 * @param id The client id
 * @param name The client name
 * @param data The client data
 */
Client::Client(string id, string name, sio::message::ptr data) : m_id(move(id)), m_name(move(name)), m_data(move(data))
{
}

Client::Client(const sio::message::ptr& message)
{
    if (isValid(message))
    {
        auto id = message->get_map()["id"];
        auto name = message->get_map()["name"];
        auto data = message->get_map()["data"];

        m_id = id->get_string();
        m_name = name->get_string();
        m_data = data;
    }
}

bool Client::isValid(const sio::message::ptr& message)
{
    if (message->get_flag() != sio::message::flag_object)
    {
        return false;
    }
    if (message->get_map().find("id") == message->get_map().end() ||
        message->get_map().find("name") == message->get_map().end() ||
        message->get_map().find("data") == message->get_map().end())
    {
        return false;
    }

    auto id = message->get_map()["id"];
    auto name = message->get_map()["name"];
    return id->get_flag() == sio::message::flag_string && name->get_flag() == sio::message::flag_string;
}

/**
 * @brief Creates a room client with the specified values.
 *
 * @param id The client id
 * @param name The client name
 * @param data The client data
 * @param isConnected Indicates if the client is connected (RTCPeerConnection)
 */
RoomClient::RoomClient(string id, string name, sio::message::ptr data, bool isConnected)
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
