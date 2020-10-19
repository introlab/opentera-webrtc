#include <OpenteraWebrtcNativeClient/Configurations/SignallingServerConfiguration.h>

using namespace introlab;
using namespace std;

SignallingServerConfiguration::SignallingServerConfiguration(const std::string& url, const std::string& clientName,
        sio::message::ptr clientData, const std::string& room, const std::string& password) :
        m_url(url), m_clientName(clientName), m_clientData(clientData), m_room(room), m_password(password)
{
}
