#include <OpenteraWebrtcNativeClient/Configurations/SignallingServerConfiguration.h>

using namespace introlab;
using namespace std;

SignallingServerConfiguration::SignallingServerConfiguration(const string& url, const string& clientName,
        sio::message::ptr clientData, const string& room, const string& password) :
        m_url(url), m_clientName(clientName), m_clientData(clientData), m_room(room), m_password(password)
{
}
