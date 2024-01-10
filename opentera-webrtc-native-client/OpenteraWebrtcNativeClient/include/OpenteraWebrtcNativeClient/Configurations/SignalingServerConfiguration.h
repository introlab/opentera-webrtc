#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_SIGNALING_SERVER_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_SIGNALING_SERVER_CONFIGURATION_H

#include <nlohmann/json.hpp>

#include <string>

namespace opentera
{
    /**
     * @brief Represents a signaling server configuration.
     */
    class SignalingServerConfiguration
    {
        std::string m_url;
        std::string m_clientName;
        nlohmann::json m_clientData;
        std::string m_room;
        std::string m_password;

        SignalingServerConfiguration(
            std::string&& url,
            std::string&& clientName,
            nlohmann::json&& clientData,
            std::string&& room,
            std::string&& password);

    public:
        SignalingServerConfiguration(const SignalingServerConfiguration& other) = default;
        SignalingServerConfiguration(SignalingServerConfiguration&& other) = default;
        virtual ~SignalingServerConfiguration() = default;

        static SignalingServerConfiguration create(std::string url, std::string clientName, std::string room);
        static SignalingServerConfiguration
            createWithData(std::string url, std::string clientName, nlohmann::json clientData, std::string room);
        static SignalingServerConfiguration
            create(std::string url, std::string clientName, std::string room, std::string password);
        static SignalingServerConfiguration createWithData(
            std::string url,
            std::string clientName,
            nlohmann::json clientData,
            std::string room,
            std::string password);

        [[nodiscard]] const std::string& url() const;
        [[nodiscard]] const std::string& clientName() const;
        [[nodiscard]] const nlohmann::json& clientData() const;
        [[nodiscard]] const std::string& room() const;
        [[nodiscard]] const std::string& password() const;

        SignalingServerConfiguration& operator=(const SignalingServerConfiguration& other) = default;
        SignalingServerConfiguration& operator=(SignalingServerConfiguration&& other) = default;
    };

    /**
     * @brief Creates an signaling server configuration with the specified values.
     *
     * @param url The signaling server URL
     * @param clientName The client name
     * @param room The room name
     * @return A signaling server configuration with the specified values
     */
    inline SignalingServerConfiguration
        SignalingServerConfiguration::create(std::string url, std::string clientName, std::string room)
    {
        return {std::move(url), std::move(clientName), nlohmann::json{}, std::move(room), ""};
    }

    /**
     * @brief Creates an signaling server configuration with the specified values.
     *
     * @param url The signaling server URL
     * @param clientName The client name
     * @param clientData The client data
     * @param room The room name
     * @return A signaling server configuration with the specified values
     */
    inline SignalingServerConfiguration SignalingServerConfiguration::createWithData(
        std::string url,
        std::string clientName,
        nlohmann::json clientData,
        std::string room)
    {
        return {std::move(url), std::move(clientName), std::move(clientData), std::move(room), ""};
    }

    /**
     * @brief Creates an signaling server configuration with the specified values.
     *
     * @param url The signaling server URL
     * @param clientName The client name
     * @param room The room name
     * @param password The signaling server password
     * @return A signaling server configuration with the specified values
     */
    inline SignalingServerConfiguration SignalingServerConfiguration::create(
        std::string url,
        std::string clientName,
        std::string room,
        std::string password)
    {
        return {std::move(url), std::move(clientName), nlohmann::json{}, std::move(room), std::move(password)};
    }

    /**
     * @brief Creates an signaling server configuration with the specified values.
     *
     * @param url The signaling server URL
     * @param clientName The client name
     * @param clientData The client data
     * @param room The room name
     * @param password The signaling server password
     * @return A signaling server configuration with the specified values
     */
    inline SignalingServerConfiguration SignalingServerConfiguration::createWithData(
        std::string url,
        std::string clientName,
        nlohmann::json clientData,
        std::string room,
        std::string password)
    {
        return {std::move(url), std::move(clientName), std::move(clientData), std::move(room), std::move(password)};
    }

    /**
     * @brief Returns the signaling server URL.
     * @return The signaling server URL
     */
    inline const std::string& SignalingServerConfiguration::url() const { return m_url; }

    /**
     * @brief Returns the client name.
     * @return The client name
     */
    inline const std::string& SignalingServerConfiguration::clientName() const { return m_clientName; }

    /**
     * @brief Returns the client data.
     * @return The client data
     */
    inline const nlohmann::json& SignalingServerConfiguration::clientData() const { return m_clientData; }

    /**
     * @brief Returns the room name.
     * @return The room name
     */
    inline const std::string& SignalingServerConfiguration::room() const { return m_room; }

    /**
     * @brief Returns the signaling server password.
     * @return The signaling server password
     */
    inline const std::string& SignalingServerConfiguration::password() const { return m_password; }
}

#endif
