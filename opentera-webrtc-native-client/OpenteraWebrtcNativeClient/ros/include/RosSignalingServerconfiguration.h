//
// Created by cedric on 20-11-18.
//

#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_ROS_SIGNALING_SERVER_CONFIGURATION_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_ROS_SIGNALING_SERVER_CONFIGURATION_H

#include <OpenteraWebrtcNativeClient/Configurations/SignallingServerConfiguration.h>

namespace introlab {

    /**
     * @brief Utility to build signaling server configuration from ROS parameter server
     */
    class RosSignalingServerConfiguration: public SignallingServerConfiguration
    {
    public:
        static SignallingServerConfiguration fromRosParam(const std::string& defaultClientName);
    };
}

#endif //OPENTERA_WEBRTC_NATIVE_CLIENT_ROS_SIGNALING_SERVER_CONFIGURATION_H
