#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_DATA_CHANNEL_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_DATA_CHANNEL_CLIENT_H

#include <OpenteraWebrtcNativeClient/WebrtcClient.h>
#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>
#include <OpenteraWebrtcNativeClient/Configurations/DataChannelConfiguration.h>

#include <api/data_channel_interface.h>

namespace opentera
{
    /**
     * @brief Represents a client for data channel communication.
     */
    class DataChannelClient : public WebrtcClient
    {
        DataChannelConfiguration m_dataChannelConfiguration;

        std::function<void(const Client&)> m_onDataChannelOpened;
        std::function<void(const Client&)> m_onDataChannelClosed;
        std::function<void(const Client&, const std::string&)> m_onDataChannelError;
        std::function<void(const Client&, const uint8_t*, std::size_t)> m_onDataChannelMessageBinary;
        std::function<void(const Client&, const std::string&)> m_onDataChannelMessageString;

    public:
        DataChannelClient(
            SignalingServerConfiguration signalingServerConfiguration,
            WebrtcConfiguration webrtcConfiguration,
            DataChannelConfiguration dataChannelConfiguration);
        ~DataChannelClient() override = default;

        DECLARE_NOT_COPYABLE(DataChannelClient);
        DECLARE_NOT_MOVABLE(DataChannelClient);

        bool sendTo(const uint8_t* data, std::size_t size, const std::vector<std::string>& ids);
        bool sendTo(const std::string& message, const std::vector<std::string>& ids);
        bool sendToAll(const uint8_t* data, std::size_t size);
        bool sendToAll(const std::string& message);

        void setOnDataChannelOpened(const std::function<void(const Client&)>& callback);
        void setOnDataChannelClosed(const std::function<void(const Client&)>& callback);
        void setOnDataChannelError(const std::function<void(const Client&, const std::string&)>& callback);
        void setOnDataChannelMessageBinary(
            const std::function<void(const Client&, const uint8_t*, std::size_t)>& callback);
        void setOnDataChannelMessageString(const std::function<void(const Client&, const std::string&)>& callback);

    protected:
        bool sendTo(const webrtc::DataBuffer& buffer, const std::vector<std::string>& ids);
        bool sendToAll(const webrtc::DataBuffer& buffer);

        std::unique_ptr<PeerConnectionHandler>
            createPeerConnectionHandler(const std::string& id, const Client& peerClient, bool isCaller) override;
    };

    /**
     * @brief Sends binary data to the specified clients.
     *
     * @param data The binary data
     * @param size The binary data size
     * @param ids The client ids
     */
    inline bool DataChannelClient::sendTo(const uint8_t* data, size_t size, const std::vector<std::string>& ids)
    {
        return sendTo(webrtc::DataBuffer(rtc::CopyOnWriteBuffer(data, size), true), ids);
    }

    /**
     * @brief Sends a string message to the specified clients.
     *
     * @param message The string message
     * @param ids The client ids
     */
    inline bool DataChannelClient::sendTo(const std::string& message, const std::vector<std::string>& ids)
    {
        return sendTo(webrtc::DataBuffer(message), ids);
    }

    /**
     * @brief Sends binary data to all clients.
     *
     * @param data The binary data
     * @param size The binary data size
     */
    inline bool DataChannelClient::sendToAll(const uint8_t* data, size_t size)
    {
        return sendToAll(webrtc::DataBuffer(rtc::CopyOnWriteBuffer(data, size), true));
    }

    /**
     * @brief Sends a string message to all clients.
     *
     * @param message The string message
     */
    inline bool DataChannelClient::sendToAll(const std::string& message)
    {
        return sendToAll(webrtc::DataBuffer(message));
    }

    /**
     * @brief Sets the callback that is called when a data channel opens.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - client: The client of the data channel that opens
     * @endparblock
     *
     * @param callback The callback
     */
    inline void DataChannelClient::setOnDataChannelOpened(const std::function<void(const Client&)>& callback)
    {
        callSync(getInternalClientThread(), [this, &callback]() { m_onDataChannelOpened = callback; });
    }

    /**
     * @brief Sets the callback that is called when a data channel closes.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - client: The client of the data channel that closes
     * @endparblock
     *
     * @param callback The callback
     */
    inline void DataChannelClient::setOnDataChannelClosed(const std::function<void(const Client&)>& callback)
    {
        callSync(getInternalClientThread(), [this, &callback]() { m_onDataChannelClosed = callback; });
    }

    /**
     * @brief Sets the callback that is called when a data channel error occurs.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - client: The client of the data channel error
     * - error: The error message
     * @endparblock
     *
     * @param callback The callback
     */
    inline void
        DataChannelClient::setOnDataChannelError(const std::function<void(const Client&, const std::string&)>& callback)
    {
        callSync(getInternalClientThread(), [this, &callback]() { m_onDataChannelError = callback; });
    }

    /**
     * @brief Sets the callback that is called when binary data are received.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - client: The client the binary data are from
     * - data: The binary data
     * - dataSize: The binary data size
     * @endparblock
     *
     * @param callback The callback
     */
    inline void DataChannelClient::setOnDataChannelMessageBinary(
        const std::function<void(const Client&, const uint8_t*, std::size_t)>& callback)
    {
        callSync(getInternalClientThread(), [this, &callback]() { m_onDataChannelMessageBinary = callback; });
    }

    /**
     * @brief Sets the callback that is called when a string message is received.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - client: The client the binary data is from
     * - message: The string message
     * @endparblock
     *
     * @param callback The callback
     */
    inline void DataChannelClient::setOnDataChannelMessageString(
        const std::function<void(const Client&, const std::string&)>& callback)
    {
        callSync(getInternalClientThread(), [this, &callback]() { m_onDataChannelMessageString = callback; });
    }
}

#endif
