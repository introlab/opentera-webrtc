#include <OpenteraWebrtcNativeClient/Configurations/SignalingServerConfiguration.h>

using namespace opentera;
using namespace std;

SignalingServerConfiguration::SignalingServerConfiguration(
    string&& url,
    string&& clientName,
    sio::message::ptr&& clientData,
    string&& room,
    string&& password)
    : m_url(move(url)),
      m_clientName(move(clientName)),
      m_clientData(move(clientData)),
      m_room(move(room)),
      m_password(move(password))
{
}
