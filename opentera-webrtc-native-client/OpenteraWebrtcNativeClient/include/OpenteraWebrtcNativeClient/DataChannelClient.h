#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_DATA_CHANNEL_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_DATA_CHANNEL_CLIENT_H

#include <OpenteraWebrtcNativeClient/SignallingClient.h>
#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>

#include <api/data_channel_interface.h>

namespace introlab
{
    class DataChannelClient : public SignalingClient
    {
        std::function<void(const Client&)> m_onDataChannelOpen;
        std::function<void(const Client&)> m_onDataChannelClosed;
        std::function<void(const Client&, const std::string&)> m_onDataChannelError;
        std::function<void(const Client&, const uint8_t*, std::size_t)> m_onDataChannelMessageBinary;
        std::function<void(const Client&, const std::string&)> m_onDataChannelMessageString;

    public:
        DataChannelClient(const std::string& url, const std::string& clientName, const sio::message::ptr& clientData,
                const std::string& room, const std::string& password, const std::vector<IceServer>& iceServers);
        ~DataChannelClient() override;

        DECLARE_NOT_COPYABLE(DataChannelClient);
        DECLARE_NOT_MOVABLE(DataChannelClient);

        void sendTo(const uint8_t* data, std::size_t size, const std::vector<std::string>& ids);
        void sendTo(const std::string& message, const std::vector<std::string>& ids);
        void sendToAll(const uint8_t* data, std::size_t size);
        void sendToAll(const std::string& message);

        void setOnDataChannelOpen(const std::function<void(const Client&)>& callback);
        void setOnDataChannelClosed(const std::function<void(const Client&)>& callback);
        void setOnDataChannelError(const std::function<void(const Client&, const std::string&)>& callback);
        void setOnDataChannelBinary(const std::function<void(const Client&, const uint8_t*, std::size_t)>& callback);
        void setOnDataChannelMessageString(const std::function<void(const Client&, const std::string&)>& callback);

    protected:
        void sendTo(const webrtc::DataBuffer& buffer, const std::vector<std::string>& ids);
        void sendToAll(const webrtc::DataBuffer& buffer);

        std::unique_ptr<PeerConnectionHandler> createPeerConnectionHandler(const std::string& id,
                const Client& peerClient, bool isCaller) override;
    };

    inline void DataChannelClient::setOnDataChannelOpen(const std::function<void(const Client&)>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_onDataChannelOpen = callback;
    }

    inline void DataChannelClient::setOnDataChannelClosed(const std::function<void(const Client&)>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_onDataChannelClosed = callback;
    }

    inline void DataChannelClient::setOnDataChannelError(
            const std::function<void(const Client&, const std::string&)>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_onDataChannelError = callback;
    }

    inline void DataChannelClient::setOnDataChannelBinary(
            const std::function<void(const Client&, const uint8_t*, std::size_t)>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_onDataChannelMessageBinary = callback;
    }

    inline void DataChannelClient::setOnDataChannelMessageString(
            const std::function<void(const Client&, const std::string&)>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_onDataChannelMessageString = callback;
    }
}

#endif
