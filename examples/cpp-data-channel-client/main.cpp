#include <OpenteraWebrtcNativeClient/DataChannelClient.h>

#include <iostream>

using namespace opentera;
using namespace std;

int main(int argc, char* argv[])
{
    vector<IceServer> iceServers;
    if (!IceServer::fetchFromServer("http://localhost:8080/iceservers", "abc", iceServers))
    {
        cout << "IceServer::fetchFromServer failed" << endl;
        iceServers.clear();
    }

    auto signalingServerConfiguration =
        SignalingServerConfiguration::create("ws://localhost:8080/signaling", "C++", "chat", "abc");
    auto webrtcConfiguration = WebrtcConfiguration::create(iceServers);
    auto dataChannelConfiguration = DataChannelConfiguration::create();
    DataChannelClient client(signalingServerConfiguration, webrtcConfiguration, dataChannelConfiguration);

    client.setOnSignalingConnectionOpened(
        []()
        {
            // This callback is called from the internal client thread.
            cout << "OnSignalingConnectionOpened" << endl;
        });
    client.setOnSignalingConnectionClosed(
        []()
        {
            // This callback is called from the internal client thread.
            cout << "OnSignalingConnectionClosed" << endl;
        });
    client.setOnSignalingConnectionError(
        [](const string& error)
        {
            // This callback is called from the internal client thread.
            cout << "OnSignalingConnectionError:" << endl << "\t" << error;
        });

    client.setOnRoomClientsChanged(
        [](const vector<RoomClient>& roomClients)
        {
            // This callback is called from the internal client thread.
            cout << "OnRoomClientsChanged:" << endl;
            for (const auto& c : roomClients)
            {
                cout << "\tid=" << c.id() << ", name=" << c.name() << ", isConnected=" << c.isConnected() << endl;
            }
        });

    client.setOnClientConnected(
        [](const Client& client)
        {
            // This callback is called from the internal client thread.
            cout << "OnClientConnected:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        });
    client.setOnClientDisconnected(
        [](const Client& client)
        {
            // This callback is called from the internal client thread.
            cout << "OnClientDisconnected:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        });
    client.setOnClientConnectionFailed(
        [](const Client& client)
        {
            // This callback is called from the internal client thread.
            cout << "OnClientConnectionFailed:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        });

    client.setOnError(
        [](const string& error)
        {
            // This callback is called from the internal client thread.
            cout << "error:" << endl;
            cout << "\t" << error << endl;
        });

    client.setLogger(
        [](const string& message)
        {
            // This callback is called from the internal client thread.
            cout << "log:" << endl;
            cout << "\t" << message << endl;
        });

    client.setOnDataChannelOpened(
        [](const Client& client)
        {
            // This callback is called from the internal client thread.
            cout << "OnDataChannelOpened:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        });
    client.setOnDataChannelClosed(
        [](const Client& client)
        {
            // This callback is called from the internal client thread.
            cout << "OnDataChannelClosed:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        });
    client.setOnDataChannelError(
        [](const Client& client, const string& error)
        {
            // This callback is called from the internal client thread.
            cout << "OnDataChannelError:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
            cout << "\t" << error << endl;
        });
    client.setOnDataChannelMessageString(
        [](const Client& client, const string& message)
        {
            // This callback is called from the internal client thread.
            cout << "setOnDataChannelMessageString:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
            cout << "\t" << message << endl;
        });

    client.connect();

    cin.get();

    return 0;
}
