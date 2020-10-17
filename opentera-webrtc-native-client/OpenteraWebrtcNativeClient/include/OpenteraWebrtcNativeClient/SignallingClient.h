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
    template<class T, class ... Types>
    void invokeIfCallable(std::function<T> f, Types... args)
    {
        if (f)
        {
            f(args...);
        }
    }

    class SignalingClient
    {
        std::string m_url;
        std::string m_clientName;
        sio::message::ptr m_clientData;
        std::string m_room;
        std::string m_password;
        std::string m_iceServers;

        std::recursive_mutex m_mutex;
        sio::client m_sio;
        bool m_hasClosePending;

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

        Client getClient(const std::string& id);
        bool isConnected();
        bool isRtcConnected();
        std::string id();

        std::vector<std::string> getConnectedRoomClientIds();
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
    };

    inline Client SignalingClient::getClient(const std::string& id)
    {
        return static_cast<Client>(m_roomClientsById.at(id));
    }

    inline bool SignalingClient::isConnected()
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        return m_sio.opened();
    }

    inline bool SignalingClient::isRtcConnected()
    {
        return !m_peerConnectionsById.empty();
    }

    inline std::string SignalingClient::id()
    {
        std::lock_guard<std::recursive_mutex> lock(m_mutex);
        return m_hasClosePending ? "" : m_sio.get_sessionid();
    }

    inline void SignalingClient::setOnSignallingConnectionOpen(const std::function<void()>& callback)
    {
        m_onSignallingConnectionOpen = callback;
    }

    inline void SignalingClient::setOnSignallingConnectionClosed(const std::function<void()>& callback)
    {
        m_onSignallingConnectionClosed = callback;
    }

    inline void SignalingClient::setOnSignallingConnectionError(const std::function<void(const std::string&)>& callback)
    {
        m_onSignallingConnectionError = callback;
    }

    inline void SignalingClient::setOnRoomClientsChanged(
            const std::function<void(const std::vector<RoomClient>&)>& callback)
    {
        m_onRoomClientsChanged = callback;
    }

    inline void SignalingClient::setCallAcceptor(const std::function<bool(const Client&)>& callback)
    {
        m_callAcceptor = callback;
    }

    inline void SignalingClient::setOnCallRejected(const std::function<void(const Client&)>& callback)
    {
        m_onCallRejected = callback;
    }
}

#endif
