#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_WEBRTC_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CONFIGURATIONS_WEBRTC_CONFIGURATION_H

#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

#include <vector>

namespace opentera
{
    /**
     * @brief Represents a WebRTC peer connection configuration.
     */
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

        [[nodiscard]] const std::vector<IceServer>& iceServers() const;

        explicit operator webrtc::PeerConnectionInterface::RTCConfiguration() const;

        WebrtcConfiguration& operator=(const WebrtcConfiguration& other) = default;
        WebrtcConfiguration& operator=(WebrtcConfiguration&& other) = default;
    };

    /**
     * @brief Creates a WebRTC peer connection configuration with default values.
     * @return A WebRTC peer connection configuration with default values
     */
    inline WebrtcConfiguration WebrtcConfiguration::create() { return WebrtcConfiguration({}); }

    /**
     * @brief Creates a WebRTC peer connection configuration with the specified value.
     * @param iceServers The ice servers
     * @return A WebRTC peer connection configuration with the specified value
     */
    inline WebrtcConfiguration WebrtcConfiguration::create(std::vector<IceServer> iceServers)
    {
        return WebrtcConfiguration(std::move(iceServers));
    }

    /**
     * Returns the ice servers.
     * @return The ice servers
     */
    inline const std::vector<IceServer>& WebrtcConfiguration::iceServers() const { return m_iceServers; }
}

#endif
