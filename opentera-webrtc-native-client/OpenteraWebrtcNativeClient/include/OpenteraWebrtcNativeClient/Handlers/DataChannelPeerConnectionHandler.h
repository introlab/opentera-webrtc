#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_FUNCTIONAL_DATA_CHANNEL_OBSERVER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_FUNCTIONAL_DATA_CHANNEL_OBSERVER_H

#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>

#include <api/data_channel_interface.h>

#include <functional>

namespace introlab
{
    class DataChannelPeerConnectionHandler : public PeerConnectionHandler, public webrtc::DataChannelObserver
    {
        std::function<void(const Client&)> m_onDataChannelOpen;
        std::function<void(const Client&)> m_onDataChannelClosed;
        std::function<void(const Client&, const std::string&)> m_onDataChannelError;
        std::function<void(const Client&, const uint8_t*, std::size_t)> m_onDataChannelMessageBinary;
        std::function<void(const Client&, const std::string&)> m_onDataChannelMessageString;

        rtc::scoped_refptr<webrtc::DataChannelInterface> m_dataChannel;

    public:
        DataChannelPeerConnectionHandler(const std::string& id,
                const Client& peerClient,
                bool isCaller,
                const std::function<void(const std::string&, sio::message::ptr)>& sendEvent,
                const std::function<void(const std::string&)>& onError,
                const std::function<void(const Client&)>& onDataChannelOpen,
                const std::function<void(const Client&)>& onDataChannelClosed,
                const std::function<void(const Client&, const std::string&)>& onDataChannelError,
                const std::function<void(const Client&, const uint8_t*, std::size_t)>& onDataChannelMessageBinary,
                const std::function<void(const Client&, const std::string&)>& onDataChannelMessageString);

        ~DataChannelPeerConnectionHandler() override = default;

        void send(const webrtc::DataBuffer& buffer);

        // Observer methods
        void OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> dataChannel) override;

        void OnStateChange() override;
        void OnMessage(const webrtc::DataBuffer& buffer) override;

        void AddRef() const override;
        rtc::RefCountReleaseStatus Release() const override;
    };
}

#endif
