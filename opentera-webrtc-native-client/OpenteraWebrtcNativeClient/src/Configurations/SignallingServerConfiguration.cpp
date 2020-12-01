#include <OpenteraWebrtcNativeClient/Configurations/SignallingServerConfiguration.h>

using namespace introlab;
using namespace std;

SignallingServerConfiguration::SignallingServerConfiguration(string&& url, string&& clientName,
        sio::message::ptr&& clientData, string&& room, string&& password) :
        m_url(move(url)), m_clientName(move(clientName)), m_clientData(move(clientData)), m_room(move(room)),
        m_password(move(password))
{
}
