#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_FUNCTIONAL_PEER_CONNECTION_OBSERVER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_FUNCTIONAL_PEER_CONNECTION_OBSERVER_H

#include <OpenteraWebrtcNativeClient/Utils/Client.h>

#include <sio_message.h>

#include <api/peer_connection_interface.h>

#include <functional>

namespace opentera
{
    class CreateSessionDescriptionObserverHelper : public webrtc::CreateSessionDescriptionObserver
    {
    public:
        void OnSuccess(webrtc::SessionDescriptionInterface* desc) final;
        void OnFailure(webrtc::RTCError error) final;

        virtual void OnCreateSessionDescriptionObserverSuccess(webrtc::SessionDescriptionInterface* desc) = 0;
        virtual void OnCreateSessionDescriptionObserverFailure(webrtc::RTCError error) = 0;
    };

    class SetSessionDescriptionObserverHelper : public webrtc::SetSessionDescriptionObserver
    {
    public:
        void OnSuccess() final;
        void OnFailure(webrtc::RTCError error) final;

        virtual void OnSetSessionDescriptionObserverSuccess() = 0;
        virtual void OnSetSessionDescriptionObserverFailure(webrtc::RTCError error) = 0;
    };


    class PeerConnectionHandler : public webrtc::PeerConnectionObserver,
            public CreateSessionDescriptionObserverHelper,
            public SetSessionDescriptionObserverHelper
    {
    protected:
        std::string m_id;
        Client m_peerClient;
        bool m_isCaller;
        std::function<void(const std::string&, const sio::message::ptr&)> m_sendEvent;
        std::function<void(const std::string&)> m_onError;
        std::function<void(const Client&)> m_onClientConnected;
        std::function<void(const Client&)> m_onClientDisconnected;

        rtc::scoped_refptr<webrtc::PeerConnectionInterface> m_peerConnection;

        bool m_onClientDisconnectedCalled;

    public:
        PeerConnectionHandler(std::string&& id,
                Client&& peerClient,
                bool isCaller,
                std::function<void(const std::string&, const sio::message::ptr&)>&& sendEvent,
                std::function<void(const std::string&)>&& onError,
                std::function<void(const Client&)>&& onClientConnected,
                std::function<void(const Client&)>&& onClientDisconnected);
        ~PeerConnectionHandler() override;

        virtual void setPeerConnection(const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection);

        void makePeerCall();
        void receivePeerCall(const std::string& sdp);
        void receivePeerCallAnswer(const std::string& sdp);
        void receiveIceCandidate(const std::string& sdpMid, int sdpMLineIndex, const std::string& sdp);

        // Observer methods
        void OnConnectionChange(webrtc::PeerConnectionInterface::PeerConnectionState newState) override;
        void OnIceCandidate(const webrtc::IceCandidateInterface* candidate) override;

        void OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> dataChannel) override;

        void OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override;
        void OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override;

        void OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState newState) override;
        void OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState newState) override;

        void OnCreateSessionDescriptionObserverSuccess(webrtc::SessionDescriptionInterface* desc) override;
        void OnCreateSessionDescriptionObserverFailure(webrtc::RTCError error) override;

        void OnSetSessionDescriptionObserverSuccess() override;
        void OnSetSessionDescriptionObserverFailure(webrtc::RTCError error) override;

        void AddRef() const override;
        rtc::RefCountReleaseStatus Release() const override;
    };
}

#endif
