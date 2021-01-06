#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_SIGNALING_SERVER_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_SIGNALING_SERVER_CONFIGURATION_H

#include <sio_message.h>

#include <string>

namespace opentera
{
    class SignalingServerConfiguration
    {
        std::string m_url;
        std::string m_clientName;
        sio::message::ptr m_clientData;
        std::string m_room;
        std::string m_password;

        SignalingServerConfiguration(std::string&& url, std::string&& clientName, sio::message::ptr&& clientData,
                std::string&& room, std::string&& password);

    public:
        SignalingServerConfiguration(const SignalingServerConfiguration& other) = default;
        SignalingServerConfiguration(SignalingServerConfiguration&& other) = default;
        virtual ~SignalingServerConfiguration() = default;

        static SignalingServerConfiguration create(std::string url, std::string clientName, std::string room);
        static SignalingServerConfiguration create(std::string url, std::string clientName,
                sio::message::ptr clientData, std::string room);
        static SignalingServerConfiguration create(std::string url, std::string clientName, std::string room,
                std::string password);
        static SignalingServerConfiguration create(std::string url, std::string clientName,
                sio::message::ptr clientData, std::string room, std::string password);

        const std::string& url() const;
        const std::string& clientName() const;
        sio::message::ptr clientData() const;
        const std::string& room() const;
        const std::string& password() const;

        SignalingServerConfiguration& operator=(const SignalingServerConfiguration& other) = default;
        SignalingServerConfiguration& operator=(SignalingServerConfiguration&& other) = default;
    };

    inline SignalingServerConfiguration SignalingServerConfiguration::create(std::string url, std::string clientName,
            std::string room)
    {
        return SignalingServerConfiguration(std::move(url), std::move(clientName), sio::null_message::create(),
                std::move(room), "");
    }

    inline SignalingServerConfiguration SignalingServerConfiguration::create(std::string url, std::string clientName,
            sio::message::ptr clientData, std::string room)
    {
        return SignalingServerConfiguration(std::move(url), std::move(clientName), std::move(clientData),
                std::move(room), "");
    }

    inline SignalingServerConfiguration SignalingServerConfiguration::create(std::string url, std::string clientName,
            std::string room, std::string password)
    {
        return SignalingServerConfiguration(std::move(url), std::move(clientName), sio::null_message::create(),
                std::move(room), std::move(password));
    }

    inline SignalingServerConfiguration SignalingServerConfiguration::create(std::string url,
            std::string clientName, sio::message::ptr clientData, std::string room, std::string password)
    {
        return SignalingServerConfiguration(std::move(url), std::move(clientName), std::move(clientData),
                std::move(room), std::move(password));
    }

    inline const std::string& SignalingServerConfiguration::url() const
    {
        return m_url;
    }

    inline const std::string& SignalingServerConfiguration::clientName() const
    {
        return m_clientName;
    }

    inline sio::message::ptr SignalingServerConfiguration::clientData() const
    {
        return m_clientData;
    }

    inline const std::string& SignalingServerConfiguration::room() const
    {
        return m_room;
    }

    inline const std::string& SignalingServerConfiguration::password() const
    {
        return m_password;
    }
}

#endif

