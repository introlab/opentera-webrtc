#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_WEBRTC_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_WEBRTC_CONFIGURATION_H

#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

#include <vector>

namespace introlab
{
    class WebrtcConfiguration
    {
        std::vector<IceServer> m_iceServers;

        explicit WebrtcConfiguration(std::vector<IceServer>&& iceServers);

    public:
        WebrtcConfiguration(const WebrtcConfiguration& other) = default;
        WebrtcConfiguration(WebrtcConfiguration&& other) = default;
        virtual ~WebrtcConfiguration() = default;

        static WebrtcConfiguration create();
        static WebrtcConfiguration create(std::vector<IceServer> iceServers);

        const std::vector<IceServer>& iceServers() const;

        operator webrtc::PeerConnectionInterface::RTCConfiguration() const;

        WebrtcConfiguration& operator=(const WebrtcConfiguration& other) = default;
        WebrtcConfiguration& operator=(WebrtcConfiguration&& other) = default;
    };

    inline WebrtcConfiguration WebrtcConfiguration::create()
    {
        return WebrtcConfiguration({});
    }

    inline WebrtcConfiguration WebrtcConfiguration::create(std::vector<IceServer> iceServers)
    {
        return WebrtcConfiguration(std::move(iceServers));
    }

    inline const std::vector<IceServer>& WebrtcConfiguration::iceServers() const
    {
        return m_iceServers;
    }
}

#endif

