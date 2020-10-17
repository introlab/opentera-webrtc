#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_DATA_CHANNEL_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_DATA_CHANNEL_CLIENT_H

#include <OpenteraWebrtcNativeClient/SignallingClient.h>
#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>

namespace introlab
{
    class DataChannelClient : public SignalingClient
    {
    public:
        DataChannelClient(const std::string& url, const std::string& clientName, const sio::message::ptr& clientData,
                        const std::string& room, const std::string& password, const std::string& iceServers);
        ~DataChannelClient() override;

        DECLARE_NOT_COPYABLE(DataChannelClient);
        DECLARE_NOT_MOVABLE(DataChannelClient);
    };
}

#endif
