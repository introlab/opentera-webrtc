#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

using namespace introlab;
using namespace std;

IceServer::IceServer(const string& url) : m_urls({url})
{
}

IceServer::IceServer(const string& url, const string& username, const string& credential) :
        m_urls({url}), m_username(username), m_credential(credential)
{
}

IceServer::IceServer(const vector<string>& urls) : m_urls(urls)
{
}

IceServer::IceServer(const vector<string>& urls, const string& username, const string& credential) :
        m_urls(urls), m_username(username), m_credential(credential)
{
}
