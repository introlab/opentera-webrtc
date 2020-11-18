//
// Created by cedric on 20-11-18.
//

#include <ros/node_handle.h>
#include <RosSignalingServerconfiguration.h>

using namespace introlab;
using namespace std;
using namespace ros;

/**
 * @brief Build a signaling server configuration from the ROS parameter server
 * Parameters are retrieved from the signaling namespace under the node private namespace
 *
 * @param defaultClientName Default name for the webrtc peer
 * @return The signaling server configuration
 */
SignallingServerConfiguration RosSignalingServerConfiguration::fromRosParam(const std::string& defaultClientName)
{
    NodeHandle pnh("~signaling");

    string serverUrl;
    pnh.param<string>("server_url", serverUrl, "http://localhost:8080");

    string clientName;
    pnh.param<string>("client_name", clientName, defaultClientName);

    string room;
    pnh.param<string>("room_name", room, "chat");

    string password;
    pnh.param<string>("room_password", password, "abc");

    return SignallingServerConfiguration::create(serverUrl, clientName, room, password);
}
