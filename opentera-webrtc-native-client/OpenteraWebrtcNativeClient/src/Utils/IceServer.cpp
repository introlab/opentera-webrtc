#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

using namespace introlab;
using namespace std;

IceServer::IceServer(string url) : m_urls({move(url)})
{
}

IceServer::IceServer(string url, string username, string credential) :
        m_urls({move(url)}), m_username(move(username)), m_credential(move(credential))
{
}

IceServer::IceServer(vector<string> urls) : m_urls(move(urls))
{
}

IceServer::IceServer(vector<string> urls, string username, string credential) :
        m_urls(move(urls)), m_username(move(username)), m_credential(move(credential))
{
}
