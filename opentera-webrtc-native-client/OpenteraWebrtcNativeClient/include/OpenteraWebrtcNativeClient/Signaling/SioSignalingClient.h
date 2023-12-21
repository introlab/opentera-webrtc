#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_SIGNALING_SIO_SIGNALING_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_SIGNALING_SIO_SIGNALING_CLIENT_H

#include <OpenteraWebrtcNativeClient/Signaling/SignalingClient.h>

#include <sio_client.h>

namespace opentera
{
    class SioSignalingClient : public SignalingClient
    {
        sio::client m_sio;
        bool m_hasClosePending;

    public:
        SioSignalingClient(SignalingServerConfiguration configuration);
        ~SioSignalingClient() override;

        DECLARE_NOT_COPYABLE(SioSignalingClient);
        DECLARE_NOT_MOVABLE(SioSignalingClient);

        void setTlsVerificationEnabled(bool isEnabled) override;

        bool isConnected() override;
        std::string sessionId() override;

        void connect() override;
        void close() override;
        void closeSync() override;

        void callAll() override;
        void callIds(const std::vector<std::string>& ids) override;
        void closeAllRoomPeerConnections() override;

        void callPeer(const std::string& toId, const std::string& sdp) override;
        void makePeerCallAnswer(const std::string& toId, const std::string& sdp) override;
        void rejectCall(const std::string& toId) override;
        void sendIceCandidate(
            const std::string& sdpMid,
            int sdpMLineIndex,
            const std::string& candidate,
            const std::string& toId) override;

    private:
        void connectSioEvents();

        void onSioConnectEvent();
        void onSioErrorEvent();
        void onSioDisconnectEvent(const sio::client::close_reason& reason);

        void onJoinRoomCallback(const sio::message::list& message);

        void onRoomClientsEvent(sio::event& event);

        void onMakePeerCallEvent(sio::event& event);
        void onPeerCallReceivedEvent(sio::event& event);
        void onPeerCallAnswerReceivedEvent(sio::event& event);
        void onCloseAllPeerConnectionsRequestReceivedEvent(sio::event& event);
        void onIceCandidateReceivedEvent(sio::event& event);
    };
}

#endif
