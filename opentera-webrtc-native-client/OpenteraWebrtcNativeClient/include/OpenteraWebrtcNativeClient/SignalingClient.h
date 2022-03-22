#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_SIGNALING_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_SIGNALING_CLIENT_H

#include <OpenteraWebrtcNativeClient/Configurations/SignalingServerConfiguration.h>
#include <OpenteraWebrtcNativeClient/Configurations/WebrtcConfiguration.h>
#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>
#include <OpenteraWebrtcNativeClient/Utils/Client.h>
#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>
#include <OpenteraWebrtcNativeClient/Handlers/PeerConnectionHandler.h>
#include <OpenteraWebrtcNativeClient/Utils/FunctionTask.h>
#include <OpenteraWebrtcNativeClient/OpenteraAudioDeviceModule.h>

#include <sio_client.h>

#include <api/peer_connection_interface.h>
#include <api/scoped_refptr.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace opentera
{
    /**
     * @brief Represents the base class of DataChannelClient and StreamClient.
     */
    class SignalingClient
    {
        std::unique_ptr<rtc::Thread> m_internalClientThread;

        WebrtcConfiguration m_webrtcConfiguration;

        sio::client m_sio;
        bool m_hasClosePending;

        std::map<std::string, Client> m_roomClientsById;
        std::vector<std::string> m_alreadyAcceptedCalls;

        std::function<void()> m_onSignalingConnectionOpened;
        std::function<void()> m_onSignalingConnectionClosed;
        std::function<void(const std::string&)> m_onSignalingConnectionError;

        std::function<void(const std::vector<RoomClient>&)> m_onRoomClientsChanged;

        std::function<bool(const Client&)> m_callAcceptor;
        std::function<void(const Client&)> m_onCallRejected;

        std::function<void(const Client&)> m_onClientConnected;
        std::function<void(const Client&)> m_onClientDisconnected;

        std::function<void(const std::string& error)> m_onError;

        std::function<void(const std::string& log)> m_logger;

        std::unique_ptr<rtc::Thread> m_networkThread;
        std::unique_ptr<rtc::Thread> m_workerThread;
        std::unique_ptr<rtc::Thread> m_signalingThread;

    protected:
        SignalingServerConfiguration m_signalingServerConfiguration;

        rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> m_peerConnectionFactory;
        std::map<std::string, std::unique_ptr<PeerConnectionHandler>> m_peerConnectionHandlersById;

        rtc::scoped_refptr<OpenteraAudioDeviceModule> m_audioDeviceModule;
        rtc::scoped_refptr<webrtc::AudioProcessing> m_audioProcessing;

    public:
        SignalingClient(
            SignalingServerConfiguration&& signalingServerConfiguration,
            WebrtcConfiguration&& webrtcConfiguration);
        virtual ~SignalingClient() = default;

        DECLARE_NOT_COPYABLE(SignalingClient);
        DECLARE_NOT_MOVABLE(SignalingClient);

        void setTlsVerificationEnabled(bool isEnabled);

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

        void setOnSignalingConnectionOpened(const std::function<void()>& callback);
        void setOnSignalingConnectionClosed(const std::function<void()>& callback);
        void setOnSignalingConnectionError(const std::function<void(const std::string&)>& callback);

        void setOnRoomClientsChanged(const std::function<void(const std::vector<RoomClient>&)>& callback);

        void setCallAcceptor(const std::function<bool(const Client&)>& callback);
        void setOnCallRejected(const std::function<void(const Client&)>& callback);

        void setOnClientConnected(const std::function<void(const Client&)>& callback);
        void setOnClientDisconnected(const std::function<void(const Client&)>& callback);

        void setOnError(const std::function<void(const std::string& error)>& callback);

        void setLogger(const std::function<void(const std::string& message)>& callback);

    protected:
        template<class T, class... Types>
        void invokeIfCallable(const std::function<T>& f, Types... args);

        void log(const std::string& message);

        virtual std::unique_ptr<PeerConnectionHandler>
            createPeerConnectionHandler(const std::string& id, const Client& peerClient, bool isCaller) = 0;

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
        void receiveIceCandidate(
            const std::string& fromId,
            const std::string& sdpMid,
            int sdpMLineIndex,
            const std::string& sdp);

        void closeAllConnections();

        bool getCallAcceptance(const std::string& id);

        std::unique_ptr<PeerConnectionHandler>
            createConnection(const std::string& peerId, const Client& peerClient, bool isCaller);
        void removeConnection(const std::string& id);
    };

    /**
     * @brief Indicates if the client is connected to the signaling server.
     * @return true if the client is connected to the signaling server
     */
    inline bool SignalingClient::isConnected()
    {
        return FunctionTask<bool>::callSync(m_internalClientThread.get(), [this]() { return m_sio.opened(); });
    }

    /**
     * @brief Indicates if the client is connected to a least one client (RTCPeerConnection).
     * @return true if the client is connected to a least one client (RTCPeerConnection)
     */
    inline bool SignalingClient::isRtcConnected()
    {
        return FunctionTask<bool>::callSync(
            m_internalClientThread.get(),
            [this]() { return !m_peerConnectionHandlersById.empty(); });
    }

    /**
     * @brief Returns the client id.
     * @return The client id
     */
    inline std::string SignalingClient::id()
    {
        return FunctionTask<std::string>::callSync(
            m_internalClientThread.get(),
            [this]() { return m_hasClosePending ? "" : m_sio.get_sessionid(); });
    }

    /**
     * @brief Returns the room client that matches with the specified id.
     * If no room client matches with the id, a default room client is returned.
     *
     * @param id The room client id
     * @return The room client that matches with the specified id
     */
    inline RoomClient SignalingClient::getRoomClient(const std::string& id)
    {
        return FunctionTask<RoomClient>::callSync(
            m_internalClientThread.get(),
            [this, &id]()
            {
                auto clientIt = m_roomClientsById.find(id);
                if (clientIt != m_roomClientsById.end())
                {
                    const auto& client = clientIt->second;
                    bool isConnected =
                        m_peerConnectionHandlersById.find(client.id()) != m_peerConnectionHandlersById.end() ||
                        client.id() == this->id();
                    return RoomClient(client, isConnected);
                }
                else
                {
                    return RoomClient();
                }
            });
    }

    /**
     * @brief Sets the callback that is called when the signaling connection opens.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @param callback The callback
     */
    inline void SignalingClient::setOnSignalingConnectionOpened(const std::function<void()>& callback)
    {
        FunctionTask<void>::callSync(
            m_internalClientThread.get(),
            [this, &callback]() { m_onSignalingConnectionOpened = callback; });
    }

    /**
     * @brief Sets the callback that is called when the signaling connection closes.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @param callback The callback
     */
    inline void SignalingClient::setOnSignalingConnectionClosed(const std::function<void()>& callback)
    {
        FunctionTask<void>::callSync(
            m_internalClientThread.get(),
            [this, &callback]() { m_onSignalingConnectionClosed = callback; });
    }

    /**
     * @brief Sets the callback that is called when a signaling connection error occurs.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - error: The error message
     * @endparblock
     *
     * @param callback The callback
     */
    inline void SignalingClient::setOnSignalingConnectionError(const std::function<void(const std::string&)>& callback)
    {
        FunctionTask<void>::callSync(
            m_internalClientThread.get(),
            [this, &callback]() { m_onSignalingConnectionError = callback; });
    }

    /**
     * @brief Sets the callback that is called when the room client changes.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - roomClients: The room clients
     * @endparblock
     *
     * @param callback The callback
     */
    inline void
        SignalingClient::setOnRoomClientsChanged(const std::function<void(const std::vector<RoomClient>&)>& callback)
    {
        FunctionTask<void>::callSync(
            m_internalClientThread.get(),
            [this, &callback]() { m_onRoomClientsChanged = callback; });
    }

    /**
     * @brief Sets the callback that is used to accept or reject a call.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - client: The client the call is from
     *
     * Callback return value:
     * - true to accept the call, false to reject the call
     * @endparblock
     *
     * @param callback The callback
     */
    inline void SignalingClient::setCallAcceptor(const std::function<bool(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]() { m_callAcceptor = callback; });
    }

    /**
     * @brief Sets the callback that is called when a call is rejected.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - client: The client that rejects the call
     * @endparblock
     *
     * @param callback The callback
     */
    inline void SignalingClient::setOnCallRejected(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(
            m_internalClientThread.get(),
            [this, &callback]() { m_onCallRejected = callback; });
    }

    /**
     * @brief Sets the callback that is called when a client peer connection opens.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - client: The client that is connected
     * @endparblock
     *
     * @param callback The callback
     */
    inline void SignalingClient::setOnClientConnected(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(
            m_internalClientThread.get(),
            [this, &callback]() { m_onClientConnected = callback; });
    }

    /**
     * @brief Sets the callback that is called when a client peer connection closes.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - client: The client that is disconnected
     * @endparblock
     *
     * @param callback The callback
     */
    inline void SignalingClient::setOnClientDisconnected(const std::function<void(const Client&)>& callback)
    {
        FunctionTask<void>::callSync(
            m_internalClientThread.get(),
            [this, &callback]() { m_onClientDisconnected = callback; });
    }

    /**
     * @brief Sets the callback that is called when an error occurs.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - error: The error message
     * @endparblock
     *
     * @param callback The callback
     */
    inline void SignalingClient::setOnError(const std::function<void(const std::string& error)>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]() { m_onError = callback; });
    }

    /**
     * @brief Sets the callback that is used to log information.
     *
     * The callback is called from the internal client thread. The callback should not block.
     *
     * @parblock
     * Callback parameters:
     * - message: The message
     * @endparblock
     *
     * @param callback The callback
     */
    inline void SignalingClient::setLogger(const std::function<void(const std::string& message)>& callback)
    {
        FunctionTask<void>::callSync(m_internalClientThread.get(), [this, &callback]() { m_logger = callback; });
    }

    template<class T, class... Types>
    void SignalingClient::invokeIfCallable(const std::function<T>& f, Types... args)
    {
        FunctionTask<void>::callAsync(
            m_internalClientThread.get(),
            [=]()
            {
                if (f)
                {
                    f(args...);
                }
            });
    }

    inline void SignalingClient::log(const std::string& message)
    {
        FunctionTask<void>::callAsync(
            m_internalClientThread.get(),
            [=]()
            {
                if (m_logger)
                {
                    m_logger(message);
                }
            });
    }

    inline rtc::Thread* SignalingClient::getInternalClientThread() { return m_internalClientThread.get(); }
}

#endif
