#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_WEBRTC_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_WEBRTC_CONFIGURATION_H

#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

#include <vector>

namespace introlab
{
    class WebrtcConfiguration
    {
        std::vector<IceServer> m_iceServers;

        explicit WebrtcConfiguration(const std::vector<IceServer>& iceServers);

    public:
        virtual ~WebrtcConfiguration() = default;

        static WebrtcConfiguration create();
        static WebrtcConfiguration create(const std::vector<IceServer>& iceServers);

        const std::vector<IceServer>& iceServers() const;

        operator webrtc::PeerConnectionInterface::RTCConfiguration() const;
    };

    inline WebrtcConfiguration WebrtcConfiguration::create()
    {
        return WebrtcConfiguration({});
    }

    inline WebrtcConfiguration WebrtcConfiguration::create(const std::vector<IceServer>& iceServers)
    {
        return WebrtcConfiguration(iceServers);
    }

    inline const std::vector<IceServer>& WebrtcConfiguration::iceServers() const
    {
        return m_iceServers;
    }
}

#endif

