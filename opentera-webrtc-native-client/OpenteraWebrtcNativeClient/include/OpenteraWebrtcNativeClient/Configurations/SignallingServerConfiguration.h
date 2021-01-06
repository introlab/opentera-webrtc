#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_SIGNALLING_SERVER_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_SIGNALLING_SERVER_CONFIGURATION_H

#include <sio_message.h>

#include <string>

namespace opentera
{
    class SignallingServerConfiguration
    {
        std::string m_url;
        std::string m_clientName;
        sio::message::ptr m_clientData;
        std::string m_room;
        std::string m_password;

        SignallingServerConfiguration(std::string&& url, std::string&& clientName, sio::message::ptr&& clientData,
                std::string&& room, std::string&& password);

    public:
        SignallingServerConfiguration(const SignallingServerConfiguration& other) = default;
        SignallingServerConfiguration(SignallingServerConfiguration&& other) = default;
        virtual ~SignallingServerConfiguration() = default;

        static SignallingServerConfiguration create(std::string url, std::string clientName, std::string room);
        static SignallingServerConfiguration create(std::string url, std::string clientName,
                sio::message::ptr clientData, std::string room);
        static SignallingServerConfiguration create(std::string url, std::string clientName, std::string room,
                std::string password);
        static SignallingServerConfiguration create(std::string url, std::string clientName,
                sio::message::ptr clientData, std::string room, std::string password);

        const std::string& url() const;
        const std::string& clientName() const;
        sio::message::ptr clientData() const;
        const std::string& room() const;
        const std::string& password() const;

        SignallingServerConfiguration& operator=(const SignallingServerConfiguration& other) = default;
        SignallingServerConfiguration& operator=(SignallingServerConfiguration&& other) = default;
    };

    inline SignallingServerConfiguration SignallingServerConfiguration::create(std::string url, std::string clientName,
            std::string room)
    {
        return SignallingServerConfiguration(std::move(url), std::move(clientName), sio::null_message::create(),
                std::move(room), "");
    }

    inline SignallingServerConfiguration SignallingServerConfiguration::create(std::string url, std::string clientName,
            sio::message::ptr clientData, std::string room)
    {
        return SignallingServerConfiguration(std::move(url), std::move(clientName), std::move(clientData),
                std::move(room), "");
    }

    inline SignallingServerConfiguration SignallingServerConfiguration::create(std::string url, std::string clientName,
            std::string room, std::string password)
    {
        return SignallingServerConfiguration(std::move(url), std::move(clientName), sio::null_message::create(),
                std::move(room), std::move(password));
    }

    inline SignallingServerConfiguration SignallingServerConfiguration::create(std::string url,
            std::string clientName, sio::message::ptr clientData, std::string room,
            std::string password)
    {
        return SignallingServerConfiguration(std::move(url), std::move(clientName), std::move(clientData),
                std::move(room), std::move(password));
    }

    inline const std::string& SignallingServerConfiguration::url() const
    {
        return m_url;
    }

    inline const std::string& SignallingServerConfiguration::clientName() const
    {
        return m_clientName;
    }

    inline sio::message::ptr SignallingServerConfiguration::clientData() const
    {
        return m_clientData;
    }

    inline const std::string& SignallingServerConfiguration::room() const
    {
        return m_room;
    }

    inline const std::string& SignallingServerConfiguration::password() const
    {
        return m_password;
    }
}

#endif

