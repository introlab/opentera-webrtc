#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_SIGNALING_SIGNALING_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_SIGNALING_SIGNALING_CLIENT_H

#include <OpenteraWebrtcNativeClient/Configurations/SignalingServerConfiguration.h>
#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>
#include <OpenteraWebrtcNativeClient/Utils/Client.h>

#include <string>
#include <vector>
#include <memory>
#include <functional>

namespace opentera
{
    class SignalingClient
    {
    protected:
        SignalingServerConfiguration m_configuration;

        std::function<void()> m_onSignalingConnectionOpened;
        std::function<void()> m_onSignalingConnectionClosed;
        std::function<void(const std::string&)> m_onSignalingConnectionError;

        std::function<void(const std::vector<Client>&)> m_onRoomClientsChanged;

        std::function<void(const std::string& id)> m_makePeerCall;
        std::function<void(const std::string& fromId, const std::string& sdp)> m_receivePeerCall;
        std::function<void(const std::string& fromId, const std::string& sdp)> m_receivePeerCallAnswer;
        std::function<
            void(const std::string& fromId, const std::string& sdpMid, int sdpMLineIndex, const std::string& sdp)>
            m_receiveIceCandidate;

        std::function<void(const std::string& fromId)> m_onCallRejected;
        std::function<void()> m_closeAllPeerConnections;

        std::function<void(const std::string& error)> m_onError;

    public:
        SignalingClient(SignalingServerConfiguration configuration);
        virtual ~SignalingClient();

        DECLARE_NOT_COPYABLE(SignalingClient);
        DECLARE_NOT_MOVABLE(SignalingClient);

        const std::string& room();

        virtual void setTlsVerificationEnabled(bool isEnabled) = 0;

        virtual bool isConnected() = 0;
        virtual std::string sessionId() = 0;

        virtual void connect() = 0;
        virtual void close() = 0;
        virtual void closeSync() = 0;

        virtual void callAll() = 0;
        virtual void callIds(const std::vector<std::string>& ids) = 0;
        virtual void closeAllRoomPeerConnections() = 0;

        virtual void callPeer(const std::string& toId, const std::string& sdp) = 0;
        virtual void makePeerCallAnswer(const std::string& toId, const std::string& sdp) = 0;
        virtual void rejectCall(const std::string& toId) = 0;
        virtual void sendIceCandidate(
            const std::string& sdpMid,
            int sdpMLineIndex,
            const std::string& candidate,
            const std::string& toId) = 0;

        void setOnSignalingConnectionOpened(const std::function<void()>& callback);
        void setOnSignalingConnectionClosed(const std::function<void()>& callback);
        void setOnSignalingConnectionError(const std::function<void(const std::string&)>& callback);

        void setOnRoomClientsChanged(const std::function<void(const std::vector<Client>&)>& callback);

        void setMakePeerCall(const std::function<void(const std::string& id)>& callback);
        void setReceivePeerCall(const std::function<void(const std::string& fromId, const std::string& sdp)>& callback);
        void setReceivePeerCallAnswer(
            const std::function<void(const std::string& fromId, const std::string& sdp)>& callback);
        void setReceiveIceCandidate(
            const std::function<
                void(const std::string& fromId, const std::string& sdpMid, int sdpMLineIndex, const std::string& sdp)>&
                callback);

        void setOnCallRejected(const std::function<void(const std::string& fromId)>& callback);
        void setCloseAllPeerConnections(const std::function<void()>& callback);

        void setOnError(const std::function<void(const std::string& error)>& callback);

    protected:
        template<class T, class... Types>
        void invokeIfCallable(const std::function<T>& f, Types... args);
    };

    inline const std::string& SignalingClient::room() { return m_configuration.room(); }

    inline void SignalingClient::setOnSignalingConnectionOpened(const std::function<void()>& callback)
    {
        m_onSignalingConnectionOpened = callback;
    }

    inline void SignalingClient::setOnSignalingConnectionClosed(const std::function<void()>& callback)
    {
        m_onSignalingConnectionClosed = callback;
    }

    inline void SignalingClient::setOnSignalingConnectionError(const std::function<void(const std::string&)>& callback)
    {
        m_onSignalingConnectionError = callback;
    }

    inline void
        SignalingClient::setOnRoomClientsChanged(const std::function<void(const std::vector<Client>&)>& callback)
    {
        m_onRoomClientsChanged = callback;
    }

    inline void SignalingClient::setMakePeerCall(const std::function<void(const std::string& id)>& callback)
    {
        m_makePeerCall = callback;
    }

    inline void SignalingClient::setReceivePeerCall(
        const std::function<void(const std::string& fromId, const std::string& sdp)>& callback)
    {
        m_receivePeerCall = callback;
    }

    inline void SignalingClient::setReceivePeerCallAnswer(
        const std::function<void(const std::string& fromId, const std::string& sdp)>& callback)
    {
        m_receivePeerCallAnswer = callback;
    }

    inline void SignalingClient::setReceiveIceCandidate(
        const std::function<
            void(const std::string& fromId, const std::string& sdpMid, int sdpMLineIndex, const std::string& sdp)>&
            callback)
    {
        m_receiveIceCandidate = callback;
    }

    inline void SignalingClient::setOnCallRejected(const std::function<void(const std::string& fromId)>& callback)
    {
        m_onCallRejected = callback;
    }

    inline void SignalingClient::setCloseAllPeerConnections(const std::function<void()>& callback)
    {
        m_closeAllPeerConnections = callback;
    }

    inline void SignalingClient::setOnError(const std::function<void(const std::string& error)>& callback)
    {
        m_onError = callback;
    }

    template<class T, class... Types>
    inline void SignalingClient::invokeIfCallable(const std::function<T>& f, Types... args)
    {
        if (f)
        {
            f(args...);
        }
    }
}

#endif
