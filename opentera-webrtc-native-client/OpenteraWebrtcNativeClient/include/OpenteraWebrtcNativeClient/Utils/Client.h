#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_CLIENT_H

#include <nlohmann/json.hpp>

#include <string>

namespace opentera
{
    /**
     * @brief Represents a peer client.
     */
    class Client
    {
        std::string m_id;
        std::string m_name;
        nlohmann::json m_data;

    public:
        Client() = default;
        Client(std::string id, std::string name, nlohmann::json data);
        explicit Client(const nlohmann::json& message);
        Client(const Client& other) = default;
        Client(Client&& other) = default;
        virtual ~Client() = default;

        [[nodiscard]] const std::string& id() const;
        [[nodiscard]] const std::string& name() const;
        [[nodiscard]] const nlohmann::json& data() const;

        static bool isValid(const nlohmann::json& message);

        Client& operator=(const Client& other) = default;
        Client& operator=(Client&& other) = default;

        friend bool operator==(const Client& c1, const Client& c2);
    };

    /**
     * @brief Returns the client id.
     * @return The client id
     */
    inline const std::string& Client::id() const { return m_id; }

    /**
     * @brief Returns the client name.
     * @return The client name
     */
    inline const std::string& Client::name() const { return m_name; }

    /**
     * @brief Returns the client data.
     * @return The client data
     */
    inline const nlohmann::json& Client::data() const { return m_data; }

    inline bool operator==(const Client& c1, const Client& c2)
    {
        return c1.m_id == c2.m_id && c1.m_name == c2.m_name && c1.m_data == c2.m_data;
    }

    inline bool operator!=(const Client& c1, const Client& c2) { return !(c1 == c2); }

    /**
     * @brief Represents a room client.
     */
    class RoomClient
    {
        std::string m_id;
        std::string m_name;
        nlohmann::json m_data;
        bool m_isConnected;

    public:
        RoomClient();
        RoomClient(std::string id, std::string name, nlohmann::json data, bool isConnected);
        RoomClient(const Client& client, bool isConnected);
        RoomClient(const RoomClient& other) = default;
        RoomClient(RoomClient&& other) = default;
        virtual ~RoomClient() = default;

        [[nodiscard]] const std::string& id() const;
        [[nodiscard]] const std::string& name() const;
        [[nodiscard]] const nlohmann::json& data() const;
        [[nodiscard]] bool isConnected() const;

        explicit operator Client() const;

        RoomClient& operator=(const RoomClient& other) = default;
        RoomClient& operator=(RoomClient&& other) = default;

        friend bool operator==(const RoomClient& c1, const RoomClient& c2);
    };

    /**
     * @brief Returns the client id.
     * @return The client id
     */
    inline const std::string& RoomClient::id() const { return m_id; }

    /**
     * @brief Returns the client name.
     * @return The client name
     */
    inline const std::string& RoomClient::name() const { return m_name; }

    /**
     * @brief Returns the client data.
     * @return The client data
     */
    inline const nlohmann::json& RoomClient::data() const { return m_data; }

    /**
     * @brief Indicates if the client is connected (RTCPeerConnection).
     * @return true if the client is connected (RTCPeerConnection)
     */
    inline bool RoomClient::isConnected() const { return m_isConnected; }

    inline RoomClient::operator Client() const { return {m_id, m_name, m_data}; }

    inline bool operator==(const RoomClient& c1, const RoomClient& c2)
    {
        return c1.m_id == c2.m_id && c1.m_name == c2.m_name && c1.m_data == c2.m_data &&
               c1.m_isConnected == c2.m_isConnected;
    }

    inline bool operator!=(const RoomClient& c1, const RoomClient& c2) { return !(c1 == c2); }
}

#endif
