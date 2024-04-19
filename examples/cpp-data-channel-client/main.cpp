#include <OpenteraWebrtcNativeClient/DataChannelClient.h>

#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>

using namespace opentera;
using namespace std;

atomic_bool stopped = false;

void sigIntSigTermSigQuitHandler(int sig)
{
    stopped = true;
}

int main(int argc, char* argv[])
{
    signal(SIGINT, sigIntSigTermSigQuitHandler);
    signal(SIGTERM, sigIntSigTermSigQuitHandler);
    signal(SIGQUIT, sigIntSigTermSigQuitHandler);

    vector<IceServer> iceServers;
    if (!IceServer::fetchFromServer("https://telesante.3it.usherbrooke.ca:40075/webrtc_ttop/10090/iceservers", "abc", iceServers))
    {
        cout << "IceServer::fetchFromServer failed" << endl;
        iceServers.clear();
    }

    auto signalingServerConfiguration =
        SignalingServerConfiguration::create("wss://telesante.3it.usherbrooke.ca:40075/webrtc_ttop/10090/signaling", "C++", "chat", "abc");
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
            cout << "setOnSignalingConnectionError:" << endl << "\t" << error;
        });

    client.setOnRoomClientsChanged(
        [&client](const vector<RoomClient>& roomClients)
        {
            // This callback is called from the internal client thread.
            cout << "OnRoomClientsChanged:" << endl;
            for (const auto& c : roomClients)
            {
                cout << "\tid=" << c.id() << ", name=" << c.name() << ", isConnected=" << c.isConnected() << endl;
            }
            client.callAll();
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
    int i = 0;
    while (!stopped)
    {
        i++;
        client.sendToAll("message " + to_string(i));
        this_thread::sleep_for(100ms);
    }

    return 0;
}
