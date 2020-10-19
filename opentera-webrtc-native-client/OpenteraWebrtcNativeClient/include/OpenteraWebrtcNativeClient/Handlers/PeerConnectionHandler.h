#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_FUNCTIONAL_PEER_CONNECTION_OBSERVER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_FUNCTIONAL_PEER_CONNECTION_OBSERVER_H

#include <OpenteraWebrtcNativeClient/Utils/Client.h>

#include <sio_client.h>

#include <api/peer_connection_interface.h>

#include <functional>

namespace introlab
{
    class PeerConnectionHandler : public webrtc::PeerConnectionObserver,
            public webrtc::CreateSessionDescriptionObserver
    {
    protected:
        std::string m_id;
        Client m_peerClient;
        bool m_isCaller;
        std::function<void(const std::string&, sio::message::ptr)> m_sendEvent;
        std::function<void(const std::string&)> m_onError;
        rtc::scoped_refptr<webrtc::PeerConnectionInterface> m_peerConnection;

    public:
        PeerConnectionHandler(const std::string& id,
                const Client& peerClient,
                bool isCaller,
                const std::function<void(const std::string&, sio::message::ptr)>& sendEvent,
                const std::function<void(const std::string&)>& onError);
        ~PeerConnectionHandler() override;

        void setPeerConnection(const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection);

        void makePeerCall();

        // Observer methods
        void OnConnectionChange(webrtc::PeerConnectionInterface::PeerConnectionState newState) override;
        void OnIceCandidate(const webrtc::IceCandidateInterface* candidate) override;

        void OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> dataChannel) override;

        void OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override;
        void OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) override;

        void OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState newState) override;
        void OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState newState) override;

        void OnSuccess(webrtc::SessionDescriptionInterface* desc) override;
        void OnFailure(webrtc::RTCError error) override;
    };

    inline void PeerConnectionHandler::setPeerConnection(
            const rtc::scoped_refptr<webrtc::PeerConnectionInterface> &peerConnection)
    {
        m_peerConnection = peerConnection;
    }
}

#endif
