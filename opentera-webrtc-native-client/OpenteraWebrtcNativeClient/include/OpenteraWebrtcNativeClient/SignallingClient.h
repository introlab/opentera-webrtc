#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_SIGNALING_CLIENT_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_SIGNALING_CLIENT_H

#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>
#include <OpenteraWebrtcNativeClient/Utils/Client.h>

#include <sio_client.h>

#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace introlab
{
    class SignalingClient
    {
        std::string m_url;
        std::string m_clientName;
        sio::message::ptr m_clientData;
        std::string m_room;
        std::string m_password;
        std::string m_iceServers;

        sio::client m_sio;
        bool m_hasClosePending;

        std::recursive_mutex m_sioMutex;
        std::recursive_mutex m_callbackMutex;

        std::map<std::string, Client> m_roomClientsById;
        std::map<std::string, bool> m_peerConnectionsById;

        std::function<void()> m_onSignallingConnectionOpen;
        std::function<void()> m_onSignallingConnectionClosed;
        std::function<void(const std::string&)> m_onSignallingConnectionError;

        std::function<void(const std::vector<RoomClient>&)> m_onRoomClientsChanged;

        std::function<bool(const Client&)> m_callAcceptor;
        std::function<void(const Client&)> m_onCallRejected;

    public:
        SignalingClient(const std::string& url, const std::string& clientName, const sio::message::ptr& clientData,
                const std::string& room, const std::string& password, const std::string& iceServers);
        virtual ~SignalingClient();

        DECLARE_NOT_COPYABLE(SignalingClient);
        DECLARE_NOT_MOVABLE(SignalingClient);

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

    protected:
        void connectSioEvents();

        void onSioConnectEvent();
        void onSioErrorEvent();
        void onSioDisconnectEvent(const sio::client::close_reason& reason);

        void onJoinRoomCallback(const sio::message::list& message);

        void onRoomClientsEvent(sio::event& event);

        template<class T, class ... Types>
        void invokeIfCallable(const std::function<T>& f, Types... args);
    };

    inline bool SignalingClient::isConnected()
    {
        std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
        return m_sio.opened();
    }

    inline bool SignalingClient::isRtcConnected()
    {
        std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
        return !m_peerConnectionsById.empty();
    }

    inline std::string SignalingClient::id()
    {
        std::lock_guard<std::recursive_mutex> lock(m_sioMutex);
        return m_hasClosePending ? "" : m_sio.get_sessionid();
    }

    inline RoomClient SignalingClient::getRoomClient(const std::string& id)
    {
        std::lock_guard<std::recursive_mutex> lock(m_sioMutex);

        const Client& client = m_roomClientsById.at(id);
        bool isConnected = m_peerConnectionsById.find(client.id()) != m_peerConnectionsById.end() ||
                client.id() == this->id();
        return RoomClient(client, isConnected);
    }

    inline void SignalingClient::setOnSignallingConnectionOpen(const std::function<void()>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_onSignallingConnectionOpen = callback;
    }

    inline void SignalingClient::setOnSignallingConnectionClosed(const std::function<void()>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_onSignallingConnectionClosed = callback;
    }

    inline void SignalingClient::setOnSignallingConnectionError(const std::function<void(const std::string&)>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_onSignallingConnectionError = callback;
    }

    inline void SignalingClient::setOnRoomClientsChanged(
            const std::function<void(const std::vector<RoomClient>&)>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_onRoomClientsChanged = callback;
    }

    inline void SignalingClient::setCallAcceptor(const std::function<bool(const Client&)>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_callAcceptor = callback;
    }

    inline void SignalingClient::setOnCallRejected(const std::function<void(const Client&)>& callback)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        m_onCallRejected = callback;
    }

    template<class T, class ... Types>
    void SignalingClient::invokeIfCallable(const std::function<T>& f, Types... args)
    {
        std::lock_guard<std::recursive_mutex> lock(m_callbackMutex);
        if (f)
        {
            f(args...);
        }
    }
}

#endif
