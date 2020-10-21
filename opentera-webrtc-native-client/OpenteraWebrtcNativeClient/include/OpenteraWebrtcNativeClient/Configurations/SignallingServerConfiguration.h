#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_SIGNALLING_SERVER_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_SIGNALLING_SERVER_CONFIGURATION_H

#include <sio_message.h>

#include <optional>
#include <string>

namespace introlab
{
    class SignallingServerConfiguration
    {
        std::string m_url;
        std::string m_clientName;
        sio::message::ptr m_clientData;
        std::string m_room;
        std::string m_password;

        SignallingServerConfiguration(const std::string& url, const std::string& clientName,
                sio::message::ptr clientData, const std::string& room, const std::string& password);

    public:
        virtual ~SignallingServerConfiguration() = default;

        static SignallingServerConfiguration create(const std::string& url, const std::string& clientName,
                const std::string& room);
        static SignallingServerConfiguration create(const std::string& url, const std::string& clientName,
                const sio::message::ptr& clientData, const std::string& room);
        static SignallingServerConfiguration create(const std::string& url, const std::string& clientName,
                const std::string& room, const std::string& password);
        static SignallingServerConfiguration create(const std::string& url, const std::string& clientName,
                const sio::message::ptr& clientData, const std::string& room, const std::string& password);

        const std::string& url() const;
        const std::string& clientName() const;
        sio::message::ptr clientData() const;
        const std::string& room() const;
        const std::string& password() const;
    };

    inline SignallingServerConfiguration SignallingServerConfiguration::create(const std::string& url,
            const std::string& clientName, const std::string& room)
    {
        return SignallingServerConfiguration(url, clientName, sio::null_message::create(), room, "");
    }

    inline SignallingServerConfiguration SignallingServerConfiguration::create(const std::string& url,
            const std::string& clientName, const sio::message::ptr& clientData, const std::string& room)
    {
        return SignallingServerConfiguration(url, clientName, clientData, room, "");
    }

    inline SignallingServerConfiguration SignallingServerConfiguration::create(const std::string& url,
            const std::string& clientName, const std::string& room, const std::string& password)
    {
        return SignallingServerConfiguration(url, clientName, sio::null_message::create(), room, password);
    }

    inline SignallingServerConfiguration SignallingServerConfiguration::create(const std::string& url,
            const std::string& clientName, const sio::message::ptr& clientData, const std::string& room,
            const std::string& password)
    {
        return SignallingServerConfiguration(url, clientName, clientData, room, password);
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

