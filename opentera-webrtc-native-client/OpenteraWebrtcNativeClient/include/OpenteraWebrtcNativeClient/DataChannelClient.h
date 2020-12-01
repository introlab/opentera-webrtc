#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_DATA_CHANNEL_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_DATA_CHANNEL_CLIENT_H

#include <OpenteraWebrtcNativeClient/SignallingClient.h>
#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>
#include <OpenteraWebrtcNativeClient/Configurations/DataChannelConfiguration.h>

#include <api/data_channel_interface.h>

namespace introlab
{
    class DataChannelClient : public SignallingClient
    {
        DataChannelConfiguration m_dataChannelConfiguration;

        std::function<void(const Client&)> m_onDataChannelOpen;
        std::function<void(const Client&)> m_onDataChannelClosed;
        std::function<void(const Client&, const std::string&)> m_onDataChannelError;
        std::function<void(const Client&, const uint8_t*, std::size_t)> m_onDataChannelMessageBinary;
        std::function<void(const Client&, const std::string&)> m_onDataChannelMessageString;

    public:
        DataChannelClient(SignallingServerConfiguration signallingServerConfiguration,
                WebrtcConfiguration webrtcConfiguration,
                DataChannelConfiguration dataChannelConfiguration);
        ~DataChannelClient() override = default;

        DECLARE_NOT_COPYABLE(DataChannelClient);
        DECLARE_NOT_MOVABLE(DataChannelClient);

        void sendTo(const uint8_t* data, std::size_t size, const std::vector<std::string>& ids);
        void sendTo(const std::string& message, const std::vector<std::string>& ids);
        void sendToAll(const uint8_t* data, std::size_t size);
        void sendToAll(const std::string& message);

        void setOnDataChannelOpen(const std::function<void(const Client&)>& callback);
        void setOnDataChannelClosed(const std::function<void(const Client&)>& callback);
        void setOnDataChannelError(const std::function<void(const Client&, const std::string&)>& callback);
        void setOnDataChannelMessageBinary(
                const std::function<void(const Client&, const uint8_t*, std::size_t)>& callback);
        void setOnDataChannelMessageString(const std::function<void(const Client&, const std::string&)>& callback);

    protected:
        void sendTo(const webrtc::DataBuffer& buffer, const std::vector<std::string>& ids);
        void sendToAll(const webrtc::DataBuffer& buffer);

        std::unique_ptr<PeerConnectionHandler> createPeerConnectionHandler(const std::string& id,
                const Client& peerClient, bool isCaller) override;
    };

    inline void DataChannelClient::sendTo(const uint8_t* data, size_t size, const std::vector<std::string>& ids)
    {
        sendTo(webrtc::DataBuffer(rtc::CopyOnWriteBuffer(data, size), true), ids);
    }

    inline void DataChannelClient::sendTo(const std::string& message, const std::vector<std::string>& ids)
    {
        sendTo(webrtc::DataBuffer(message), ids);
    }

    inline void DataChannelClient::sendToAll(const uint8_t* data, size_t size)
    {
        sendToAll(webrtc::DataBuffer(rtc::CopyOnWriteBuffer(data, size), true));
    }

    inline void DataChannelClient::sendToAll(const std::string& message)
    {
        sendToAll(webrtc::DataBuffer(message));
    }

    inline void DataChannelClient::setOnDataChannelOpen(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onDataChannelOpen = callback;
        });
    }

    inline void DataChannelClient::setOnDataChannelClosed(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onDataChannelClosed = callback;
        });
    }

    inline void DataChannelClient::setOnDataChannelError(
            const std::function<void(const Client&, const std::string&)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onDataChannelError = callback;
        });
    }

    inline void DataChannelClient::setOnDataChannelMessageBinary(
            const std::function<void(const Client&, const uint8_t*, std::size_t)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onDataChannelMessageBinary = callback;
        });
    }

    inline void DataChannelClient::setOnDataChannelMessageString(
            const std::function<void(const Client&, const std::string&)>& callback)
    {
        FunctionTask<void>::callSync(getInternalClientThread(), [this, &callback]()
        {
            m_onDataChannelMessageString = callback;
        });
    }
}

#endif
