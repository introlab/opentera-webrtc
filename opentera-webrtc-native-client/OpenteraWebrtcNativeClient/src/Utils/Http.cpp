#include <OpenteraWebrtcNativeClient/Utils/Http.h>

#include <httplib.h>

using namespace opentera;
using namespace std;

bool Http::get(const string& url, string& response, multimap<string, string> headers, bool verifyCertificate)
{
    string host;
    string target;

    if (!Http::splitUrl(url, host, target))
    {
        return false;
    }

    try
    {
        httplib::Client cli(host);
#if defined(__APPLE__)
        /*
            TODO : Remove the certificate path on macosx with new version of
           BoringSSL? As of now, the ca-certificates need to be installed on the
           system with the following command: brew install ca-certificates
        */
        cli.set_ca_cert_path("/usr/local/etc/ca-certificates/cert.pem");
#elif defined(_WIN32)
        /*
            TODO : Find a better solution than MSYS2 to install the certificate on
           windows. As of now, the ca-certificates need to be installed on the
           system with the following command: pacman -S ca-certificates
        */
        cli.set_ca_cert_path("C:\\msys64\\usr\\ssl\\cert.pem");
#endif
        cli.enable_server_certificate_verification(verifyCertificate);
        auto res = cli.Get(target, httplib::Headers(headers.begin(), headers.end()));
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
