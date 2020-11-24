#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_SIGNALING_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_SIGNALING_CLIENT_H

#include <OpenteraWebrtcNativeClient/Configurations/SignallingServerConfiguration.h>
#include <OpenteraWebrtcNativeClient/Configurations/WebrtcConfiguration.h>
#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>
#include <OpenteraWebrtcNativeClient/Utils/Client.h>
#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>
#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>
#include <OpenteraWebrtcNativeClient/Utils/FunctionTask.h>

#include <sio_client.h>

#include <api/peer_connection_interface.h>
#include <api/scoped_refptr.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace introlab
{
    class SignallingClient
    {
        std::unique_ptr<rtc::Thread> m_internalClientThread;

        WebrtcConfiguration m_webrtcConfiguration;

        sio::client m_sio;
        bool m_hasClosePending;

        std::map<std::string, Client> m_roomClientsById;
        std::vector<std::string> m_alreadyAcceptedCalls;

        std::function<void()> m_onSignallingConnectionOpen;
        std::function<void()> m_onSignallingConnectionClosed;
        std::function<void(const std::string&)> m_onSignallingConnectionError;

        std::function<void(const std::vector<RoomClient>&)> m_onRoomClientsChanged;

        std::function<bool(const Client&)> m_callAcceptor;
        std::function<void(const Client&)> m_onCallRejected;

        std::function<void(const Client&)> m_onClientConnected;
        std::function<void(const Client&)> m_onClientDisconnected;

        std::function<void(const std::string& error)> m_onError;

        std::unique_ptr<rtc::Thread> m_networkThread;
        std::unique_ptr<rtc::Thread> m_workerThread;
        std::unique_ptr<rtc::Thread> m_signallingThread;

    protected:
        SignallingServerConfiguration m_signallingServerConfiguration;

        rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> m_peerConnectionFactory;
        std::map<std::string, std::unique_ptr<PeerConnectionHandler>> m_peerConnectionHandlersById;

    public:
        SignallingClient(const SignallingServerConfiguration& signallingServerConfiguration,
                         const WebrtcConfiguration& webrtcConfiguration);
        virtual ~SignallingClient();

        DECLARE_NOT_COPYABLE(SignallingClient);
        DECLARE_NOT_MOVABLE(SignallingClient);

        void connect();
        void close();
        void closeSync();

        void callAll();
        void callIds(const std::vector<std::string>& ids);

        void hangUpAll();
        void closeAllRoomPeerConnections();

        bool isConnected();
        bool isRtcConnected();
        std::string id();

        std::vector<std::string> getConnectedRoomClientIds();

        RoomClient getRoomClient(const std::string& id);
        std::vector<RoomClient> getRoomClients();

        void setOnSignallingConnectionOpen(const std::function<void()>& callback);
        void setOnSignallingConnectionClosed(const std::function<void()>& callback);
        void setOnSignallingConnectionError(const std::function<void(const std::string&)>& callback);

        void setOnRoomClientsChanged(const std::function<void(const std::vector<RoomClient>&)>& callback);

        void setCallAcceptor(const std::function<bool(const Client&)>& callback);
        void setOnCallRejected(const std::function<void(const Client&)>& callback);

        void setOnClientConnected(const std::function<void(const Client&)>& callback);
        void setOnClientDisconnected(const std::function<void(const Client&)>& callback);

        void setOnError(const std::function<void(const std::string& error)>& callback);

    protected:
        template<class T, class ... Types>
        void invokeIfCallable(const std::function<T>& f, Types... args);

        virtual std::unique_ptr<PeerConnectionHandler> createPeerConnectionHandler(const std::string& id,
                const Client& peerClient, bool isCaller) = 0;

        std::function<void(const std::string&, sio::message::ptr)> getSendEventFunction();
        std::function<void(const std::string&)> getOnErrorFunction();
        std::function<void(const Client&)> getOnClientConnectedFunction();
        std::function<void(const Client&)> getOnClientDisconnectedFunction();

        rtc::Thread* getInternalClientThread();

    private:
        void connectSioEvents();

        void onSioConnectEvent();
        void onSioErrorEvent();
        void onSioDisconnectEvent(const sio::client::close_reason& reason);

        void onJoinRoomCallback(const sio::message::list& message);

        void onRoomClientsEvent(sio::event& event);

        void onMakePeerCallEvent(sio::event& event);
        void makePeerCall(const std::string& id);

        void onPeerCallReceivedEvent(sio::event& event);
        void receivePeerCall(const std::string& fromId, const std::string& sdp);

        void onPeerCallAnswerReceivedEvent(sio::event& event);
        void receivePeerCallAnswer(const std::string& fromId, const std::string& sdp);

        void onCloseAllPeerConnectionsRequestReceivedEvent(sio::event& event);

        void onIceCandidateReceivedEvent(sio::event& event);
        void receiveIceCandidate(const std::string& fromId, const std::string& sdpMid, int sdpMLineIndex,
                const std::string& sdp);

        void closeAllConnections();

        bool getCallAcceptance(const std::string& id);

        std::unique_ptr<PeerConnectionHandler> createConnection(const std::string& peerId, const Client& peerClient,
                bool isCaller);
        void removeConnection(const std::string& id);
    };

    inline bool SignallingClient::isConnected()
    {
        return FunctionTask<bool>::callSync(m_internalClientThread.get(), [this]()
        {
            return m_sio.opened();
        });
    }

    inline bool SignallingClient::isRtcConnected()
    {
        return FunctionTask<bool>::callSync(m_internalClientThread.get(), [this]()
        {
            return !m_peerConnectionHandlersById.empty();
        });
    }

    inline std::string SignallingClient::id()
    {
        return FunctionTask<std::string>::callSync(m_internalClientThread.get(), [this]()
        {
            return m_hasClosePending ? "" : m_sio.get_sessionid();
        });
    }

    inline RoomClient SignallingClient::getRoomClient(const std::string& id)
    {
        return FunctionTask<RoomClient>::callSync(m_internalClientThread.get(), [this, &id]()
        {
            auto clientIt = m_roomClientsById.find(id);
            if (clientIt != m_roomClientsById.end())
            {
                const auto& client = clientIt->second;
                bool isConnected = m_peerConnectionHandlersById.find(client.id()) != m_peerConnectionHandlersById.end() ||
                                   client.id() == this->id();
                return RoomClient(client, isConnected);
            }
            else
            {
                return RoomClient();
            }
        });

    }

    inline void SignallingClient::setOnSignallingConnectionOpen(const std::function<void()>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]()
        {
            m_onSignallingConnectionOpen = callback;
        });
    }

    inline void SignallingClient::setOnSignallingConnectionClosed(const std::function<void()>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]()
        {
            m_onSignallingConnectionClosed = callback;
        });
    }

    inline void SignallingClient::setOnSignallingConnectionError(
            const std::function<void(const std::string&)>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]()
        {
            m_onSignallingConnectionError = callback;
        });
    }

    inline void SignallingClient::setOnRoomClientsChanged(
            const std::function<void(const std::vector<RoomClient>&)>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]()
        {
            m_onRoomClientsChanged = callback;
        });
    }

    inline void SignallingClient::setCallAcceptor(const std::function<bool(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]()
        {
            m_callAcceptor = callback;
        });
    }

    inline void SignallingClient::setOnCallRejected(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]()
        {
            m_onCallRejected = callback;
        });
    }

    inline void SignallingClient::setOnClientConnected(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]()
        {
            m_onClientConnected = callback;
        });
    }

    inline void SignallingClient::setOnClientDisconnected(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]()
        {
            m_onClientDisconnected = callback;
        });
    }

    inline void SignallingClient::setOnError(const std::function<void(const std::string& error)>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]()
        {
            m_onError = callback;
        });
    }

    template<class T, class ... Types>
    void SignallingClient::invokeIfCallable(const std::function<T>& f, Types... args)
    {
        FunctionTask<void>::callAsync(m_internalClientThread.get(), [=]()
        {
            if (f)
            {
                f(args...);
            }
        });
    }

    inline rtc::Thread* SignallingClient::getInternalClientThread()
    {
        return m_internalClientThread.get();
    }
}

#endif
