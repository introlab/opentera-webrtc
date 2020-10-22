#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_CLIENT_H

#include <sio_message.h>

#include <string>

namespace introlab
{
    bool operator==(const sio::message& m1, const sio::message& m2);
    bool operator!=(const sio::message& m1, const sio::message& m2);

    class Client
    {
        std::string m_id;
        std::string m_name;
        sio::message::ptr m_data;

    public:
        Client() = default;
        Client(const std::string& id, const std::string& name, const sio::message::ptr& data);
        Client(const sio::message::ptr& message);
        virtual ~Client() = default;

        const std::string& id() const;
        const std::string& name() const;
        const sio::message::ptr& data() const;

        static bool isValid(const sio::message::ptr& message);

        friend bool operator==(const Client& c1, const Client& c2);
    };

    inline const std::string& Client::id() const
    {
        return m_id;
    }

    inline const std::string& Client::name() const
    {
        return m_name;
    }

    inline const sio::message::ptr& Client::data() const
    {
        return m_data;
    }

    inline bool operator==(const Client& c1, const Client& c2)
    {
        return c1.m_id == c2.m_id && c1.m_name == c2.m_name && *c1.m_data == *c2.m_data;
    }

    inline bool operator!=(const Client& c1, const Client& c2)
    {
        return !(c1 == c2);
    }

    class RoomClient
    {
        std::string m_id;
        std::string m_name;
        sio::message::ptr m_data;
        bool m_isConnected;

    public:
        RoomClient() = default;
        RoomClient(const std::string& id, const std::string& name, const sio::message::ptr& data, bool isConnected);
        RoomClient(const Client& client, bool isConnected);
        virtual ~RoomClient() = default;

        const std::string& id() const;
        const std::string& name() const;
        const sio::message::ptr& data() const;
        bool isConnected() const;

        explicit operator Client() const;

        friend bool operator==(const RoomClient& c1, const RoomClient& c2);
    };

    inline const std::string& RoomClient::id() const
    {
        return m_id;
    }

    inline const std::string& RoomClient::name() const
    {
        return m_name;
    }

    inline const sio::message::ptr& RoomClient::data() const
    {
        return m_data;
    }

    inline bool RoomClient::isConnected() const
    {
        return m_isConnected;
    }

    inline RoomClient::operator Client() const
    {
        return Client(m_id, m_name, m_data);
    }

    inline bool operator==(const RoomClient& c1, const RoomClient& c2)
    {
        if (c1.m_data != nullptr && c2.m_data != nullptr)
        {
            return c1.m_id == c2.m_id && c1.m_name == c2.m_name && (*c1.m_data == *c2.m_data) &&
                    c1.m_isConnected == c2.m_isConnected;
        }
        else if (c1.m_data == nullptr && c2.m_data == nullptr)
        {
            return c1.m_id == c2.m_id && c1.m_name == c2.m_name && c1.m_isConnected == c2.m_isConnected;
        }
        else
        {
            return false;
        }
    }

    inline bool operator!=(const RoomClient& c1, const RoomClient& c2)
    {
        return !(c1 == c2);
    }
}

#endif
