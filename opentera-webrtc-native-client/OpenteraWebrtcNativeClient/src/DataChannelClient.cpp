#include <OpenteraWebrtcNativeClient/DataChannelClient.h>

using namespace introlab;
using namespace std;

DataChannelClient::DataChannelClient(const string& url, const string& clientName, const sio::message::ptr& clientData,
                                 const string& room, const string& password, const string& iceServers) :
        SignalingClient(url, clientName, clientData, room, password, iceServers)
{
}

DataChannelClient::~DataChannelClient()
{
}
