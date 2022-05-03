#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_FUNCTIONAL_DATA_CHANNEL_OBSERVER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_FUNCTIONAL_DATA_CHANNEL_OBSERVER_H

#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>
#include <OpenteraWebrtcNativeClient/Configurations/DataChannelConfiguration.h>

#include <api/data_channel_interface.h>

#include <functional>

namespace opentera
{
    class DataChannelPeerConnectionHandler : public PeerConnectionHandler, public webrtc::DataChannelObserver
    {
        std::string m_room;
        DataChannelConfiguration m_dataChannelConfiguration;

        std::function<void(const Client&)> m_onDataChannelOpen;
        std::function<void(const Client&)> m_onDataChannelClosed;
        std::function<void(const Client&, const std::string&)> m_onDataChannelError;
        std::function<void(const Client&, const webrtc::DataBuffer& buffer)> m_onDataChannelMessageBinary;
        std::function<void(const Client&, const std::string&)> m_onDataChannelMessageString;

        rtc::scoped_refptr<webrtc::DataChannelInterface> m_dataChannel;

        bool m_onDataChannelClosedCalled;

    public:
        DataChannelPeerConnectionHandler(
            std::string id,
            Client peerClient,
            bool isCaller,
            std::function<void(const std::string&, const sio::message::ptr&)> sendEvent,
            std::function<void(const std::string&)> onError,
            std::function<void(const Client&)> onClientConnected,
            std::function<void(const Client&)> onClientDisconnected,
            std::string room,
            DataChannelConfiguration dataChannelConfiguration,
            std::function<void(const Client&)> onDataChannelOpen,
            std::function<void(const Client&)> onDataChannelClosed,
            std::function<void(const Client&, const std::string&)> onDataChannelError,
            std::function<void(const Client&, const webrtc::DataBuffer& buffer)> onDataChannelMessageBinary,
            std::function<void(const Client&, const std::string&)> onDataChannelMessageString);

        ~DataChannelPeerConnectionHandler() override;

        void setPeerConnection(const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection) override;

        void send(const webrtc::DataBuffer& buffer);

        // Observer methods
        void OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> dataChannel) override;

        void OnStateChange() override;
        void OnMessage(const webrtc::DataBuffer& buffer) override;

    protected:
        virtual void createAnswer();
    };
}

#endif
