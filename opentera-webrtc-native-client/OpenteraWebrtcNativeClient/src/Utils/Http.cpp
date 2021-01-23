#include <OpenteraWebrtcNativeClient/Utils/Http.h>

#include <httplib.h>

using namespace opentera;
using namespace std;

bool Http::get(const string& url, string& response, multimap<string, string> headers)
{
    string host;
    string target;

    if(!Http::splitUrl(url, host, target))
    {
        return false;
    }

    try
    {
        httplib::Client cli(host.c_str());
        auto res = cli.Get(target.c_str(), httplib::Headers(headers.begin(), headers.end()));
        if (res == nullptr || res->status != 200)
        {
            return false;
        }
        response = res->body;
    }
    catch (...)
    {
        return false;
    }
    return true;
}

bool Http::splitUrl(const string& url, string& host, string& target)
{
    size_t hostBeginPosistion = url.find("://");
    if (hostBeginPosistion == string::npos)
    {
        return false;
    }

    hostBeginPosistion += 3;
    size_t firstSlashPosition = url.find('/', hostBeginPosistion);

    if (firstSlashPosition == string::npos)
    {
        host = url;
        target = "/";
    }
    else
    {
        host = url.substr(0, firstSlashPosition);
        target = url.substr(firstSlashPosition);
    }

    return true;
}
