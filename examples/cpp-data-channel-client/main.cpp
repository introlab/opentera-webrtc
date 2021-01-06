#include <OpenteraWebrtcNativeClient/DataChannelClient.h>

#include <iostream>

using namespace opentera;
using namespace std;

int main(int argc, char* argv[])
{
    vector<IceServer> iceServers;
    if (!IceServer::fetchFromServer("", "abc", iceServers))
    {
        iceServers.clear();
    }

    auto signallingServerConfiguration =
            SignallingServerConfiguration::create("http://localhost:8080", "C++", "chat", "abc");
    auto webrtcConfiguration = WebrtcConfiguration::create(iceServers);
    auto dataChannelConfiguration = DataChannelConfiguration::create();
    DataChannelClient client(signallingServerConfiguration, webrtcConfiguration, dataChannelConfiguration);

    client.setOnSignallingConnectionOpen([]()
    {
        cout << "OnSignallingConnectionOpen" << endl;
    });
    client.setOnSignallingConnectionClosed([]()
    {
        cout << "OnSignallingConnectionClosed" << endl;
    });
    client.setOnSignallingConnectionError([](const string& error)
    {
        cout << "OnSignallingConnectionClosed:" << endl << "\t" << error;
    });

    client.setOnRoomClientsChanged([](const vector<RoomClient>& roomClients)
    {
        cout << "OnRoomClientsChanged:" << endl;
        for (const auto& c : roomClients)
        {
            cout << "\tid=" << c.id() << ", name=" << c.name() << ", isConnected=" << c.isConnected() << endl;
        }
    });

    client.setOnClientConnected([](const Client& client)
    {
        cout << "OnClientConnected:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
    });
    client.setOnClientDisconnected([](const Client& client)
    {
        cout << "OnClientDisconnected:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
    });

    client.setOnError([](const string& error)
    {
        cout << "error or warning:" << endl;
        cout << "\t" << error << endl;
    });

    client.setOnDataChannelOpen([](const Client& client)
    {
        cout << "OnDataChannelOpen:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
    });
    client.setOnDataChannelClosed([](const Client& client)
    {
        cout << "OnDataChannelClosed:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
    });
    client.setOnDataChannelError([](const Client& client, const string& error)
    {
        cout << "OnDataChannelError:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        cout << "\t" << error << endl;
    });
    client.setOnDataChannelMessageString([](const Client& client, const string& message)
    {
        cout << "OnDataChannelError:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        cout << "\t" << message << endl;
    });

    client.connect();

    cin.get();

    return 0;
}

