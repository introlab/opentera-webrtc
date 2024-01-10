#include <OpenteraWebrtcNativeClient/DataChannelClient.h>

#include <iostream>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>

using namespace opentera;
using namespace std;

constexpr chrono::milliseconds CLOSING_CONNECTION_DELAY(1000);

constexpr int MESSAGE_COUNT = 100;

atomic_bool isRunning = true;

void sigintSigtermCallbackHandler(int signum)
{
    isRunning = false;
}

template <class F>
bool waitFor(F f)
{
    constexpr chrono::milliseconds SLEEP_TIME(100);
    constexpr chrono::milliseconds TIMEOUT(10000);

    auto start = chrono::steady_clock::now();
    while (!f())
    {
        this_thread::sleep_for(SLEEP_TIME);

        if (chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start) > TIMEOUT)
        {
            return false;
        }
        else if (!isRunning)
        {
            exit(-1);
        }
    }

    return true;
}

int main(int argc, char* argv[])
{
    if (argc != 5)
    {
        cout << "Usage: CppDataChannelReliabilityTests base_url name password master(true|false)" << endl;
        return -1;
    }

    string baseUrl = argv[1];
    string name = argv[2];
    string password  = argv[3];
    bool isMaster = string(argv[4]) != "false";

    signal(SIGINT, sigintSigtermCallbackHandler);
    signal(SIGTERM, sigintSigtermCallbackHandler);

    vector<IceServer> iceServers;
    if (!IceServer::fetchFromServer(baseUrl + "/iceservers", password, iceServers))
    {
        cout << "IceServer::fetchFromServer failed" << endl;
        iceServers.clear();
    }

    cout << "Ice servers=" << endl;
    for (auto s : iceServers)
    {
        cout << "\turls=" << endl;
        for (auto u : s.urls())
        {
            cout << "\t\t" << u << endl;
        }
        cout << "\tusername=" << s.username() << endl;
        cout << "\tcredential=" << s.credential() << endl;
    }
    cout  << endl;

    string wsUrl;
    if (baseUrl.find("http://") == 0)
    {
        wsUrl = "ws://" + baseUrl.substr(7) + "/signaling";
    }
    else if (baseUrl.find("https://") == 0)
    {
        wsUrl = "wss://" + baseUrl.substr(8) + "/signaling";
    }
    else
    {
        cout << "Invalid base URL (" << baseUrl << ")" << endl;
        return -1;
    }
    auto signalingServerConfiguration =
        SignalingServerConfiguration::create(baseUrl, name, "reliability", password);
    auto webrtcConfiguration = WebrtcConfiguration::create(iceServers);
    auto dataChannelConfiguration = DataChannelConfiguration::create();
    DataChannelClient client(signalingServerConfiguration, webrtcConfiguration, dataChannelConfiguration);

    atomic_bool hasAnotherClient = false;
    atomic_bool isDataChannelOpened = false;
    atomic_int currentMessageId = 0;

    int successfulConnectionCount = 0;
    int failedConnectionCount = 0;
    int successfulMessageGroupCount = 0;
    int failedMessageGroupCount = 0;

    client.setOnSignalingConnectionError(
        [](const string& error)
        {
            cout << "OnSignalingConnectionClosed:" << endl << "\t" << error;
        });

    client.setOnRoomClientsChanged(
        [&](const vector<RoomClient>& roomClients)
        {
            hasAnotherClient = roomClients.size() > 1;
        });

    client.setOnError(
        [](const string& error)
        {
            cout << "error:" << endl;
            cout << "\t" << error << endl;
        });

    client.setOnDataChannelOpened(
        [&](const Client& client)
        {
            isDataChannelOpened = true;
        });
    client.setOnDataChannelError(
        [](const Client& client, const string& error)
        {
            cout << "OnDataChannelError:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
            cout << "\t" << error << endl;
        });
    client.setOnDataChannelMessageString(
        [&](const Client& _, const string& message)
        {
            int receivedId = stoi(message);
            currentMessageId = receivedId + 1;
            if (!client.sendToAll(to_string(currentMessageId.load())))
            {
                cout << "sendToAll failed" << endl;
            }

            cout << "receivedId=" << receivedId << endl;
        });

    client.connect();

    cout << "Connecting to the signaling server." << endl;
    if (!waitFor([&](){ return client.isConnected(); }))
    {
        cout << "Signaling server connection failed." << endl;
        return -1;
    }

    cout << "Waiting for another client." << endl;
    if (!waitFor([&](){ return hasAnotherClient.load(); }))
    {
        cout << "No other client." << endl;
        return -1;
    }

    while (isRunning)
    {
        isDataChannelOpened = false;
        if (isMaster)
        {
            client.callAll();
        }

        if (waitFor([&](){ return isDataChannelOpened.load(); }))
        {
            successfulConnectionCount++;
            currentMessageId = 0;
            if (isMaster)
            {
                if (!client.sendToAll(to_string(currentMessageId.load())))
                {
                    cout << "sendToAll failed" << endl;
                }
            }

            if (waitFor([&](){ return currentMessageId.load() >= MESSAGE_COUNT; }))
            {
                successfulMessageGroupCount++;
            }
            else
            {
                failedMessageGroupCount++;
            }
        }
        else
        {
            failedConnectionCount++;
        }

        if (isMaster)
        {
            client.closeAllRoomPeerConnections();
        }

        cout << endl << "************ Stats ************" << endl;
        cout << "\t successfulConnectionCount=" << successfulConnectionCount << endl;
        cout << "\t failedConnectionCount=" << failedConnectionCount << endl;
        cout << "\t successfulMessageGroupCount=" << successfulMessageGroupCount << endl;
        cout << "\t failedMessageGroupCount=" << failedMessageGroupCount << endl;
        cout << "************ Stats ************" << endl << endl;

        this_thread::sleep_for(CLOSING_CONNECTION_DELAY);
    }

    client.closeSync();

    return 0;
}
