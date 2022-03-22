#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_ICE_SERVER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_ICE_SERVER_H

#include <api/peer_connection_interface.h>

#include <string>
#include <vector>

namespace opentera
{
    /**
     * @brief Represents an ice server configuration.
     */
    class IceServer
    {
        std::vector<std::string> m_urls;
        std::string m_username;
        std::string m_credential;

    public:
        explicit IceServer(std::string url);
        IceServer(std::string url, std::string username, std::string credential);
        explicit IceServer(std::vector<std::string> urls);
        IceServer(std::vector<std::string> urls, std::string username, std::string credential);
        virtual ~IceServer() = default;

        const std::vector<std::string>& urls() const;
        const std::string& username() const;
        const std::string& credential() const;

        explicit operator webrtc::PeerConnectionInterface::IceServer() const;

        static bool fetchFromServer(
            const std::string& url,
            const std::string& password,
            std::vector<IceServer>& iceServers,
            bool verifyCertificate = true);
        static bool fromJson(const std::string& json, std::vector<IceServer>& iceServers);
    };

    /**
     * @brief Returns the ice server urls.
     * @return The ice server urls
     */
    inline const std::vector<std::string>& IceServer::urls() const { return m_urls; }

    /**
     * @brief Returns the ice server username.
     * @return The ice server username
     */
    inline const std::string& IceServer::username() const { return m_username; }

    /**
     * @brief Returns the ice server credential.
     * @return The ice server credential
     */
    inline const std::string& IceServer::credential() const { return m_credential; }

    inline IceServer::operator webrtc::PeerConnectionInterface::IceServer() const
    {
        webrtc::PeerConnectionInterface::IceServer iceServer;
        iceServer.urls = m_urls;
        iceServer.username = m_username;
        iceServer.password = m_credential;
        return iceServer;
    }
}

#endif
