#include <OpenteraWebrtcNativeClient/Utils/Http.h>

using namespace introlab;
using namespace boost::asio;
using namespace boost::beast;
using namespace std;

#define CHECK_BOOST_ERROR_CODE(function_call) \
    function_call ; \
    if (ec.failed()) \
    { \
        return false; \
    } \
    do {} while(false)

bool Http::get(const string& url, string& response,
        function<void(http::request<boost::beast::http::string_body>&)> requestModifier)
{
    constexpr int HttpVersion = 11;

    string host;
    string port;
    string target;

    size_t httpPosition = url.find("http://");
    size_t httpsPosition = url.find("https://");

    if (httpPosition == 0)
    {
        getHostPortTargetFromUrl(url, "80", host, port, target);
    }
    else if (httpsPosition == 0)
    {
        getHostPortTargetFromUrl(url, "443", host, port, target);
    }
    else
    {
        return false;
    }

    http::request<http::string_body> request{http::verb::get, target, HttpVersion};
    request.set(http::field::host, host);
    request.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
    requestModifier(request);

    if (httpPosition == 0)
    {
        getHttp(host, port, request, response);
    }
    else if (httpsPosition == 0)
    {
        getHttps(host, port, request, response);
    }
    return true;
}

bool Http::getHostPortTargetFromUrl(const string& url, const string& defaultPort, string& host, string& port,
        string& target)
{
    size_t hostBeginPosistion = url.find("://");
    if (hostBeginPosistion == string::npos)
    {
        return false;
    }

    hostBeginPosistion += 3;
    size_t hostPortSeparator = url.find(':', hostBeginPosistion);
    size_t firstSlashPosition = url.find('/', hostBeginPosistion);

    if (hostPortSeparator == string::npos)
    {
        port = defaultPort;
        if (firstSlashPosition == string::npos)
        {
            host = url.substr(hostBeginPosistion);
            target = "/";
        }
        else
        {
            host = url.substr(hostBeginPosistion, firstSlashPosition - hostBeginPosistion);
            target = url.substr(firstSlashPosition);
        }
    }
    else
    {
        host = url.substr(hostBeginPosistion, hostPortSeparator - hostBeginPosistion);
        hostPortSeparator += 1;
        if (firstSlashPosition == string::npos)
        {
            port = url.substr(hostPortSeparator);
            target = "/";
        }
        else
        {
            port = url.substr(hostPortSeparator, firstSlashPosition - hostPortSeparator);
            target = url.substr(firstSlashPosition);
        }
    }

    return true;
}

bool Http::getHttp(const string& host, const string& port, const http::request<http::string_body>& request,
        string& responseOutput)
{
    boost::system::error_code ec;
    using namespace boost::asio;

    io_service ioService;

    // Host name resolution
    ip::tcp::resolver resolver(ioService);
    CHECK_BOOST_ERROR_CODE(auto hostResults = resolver.resolve({host, port}, ec));

    // Connection
    boost::beast::tcp_stream socketStream(ioService);
    CHECK_BOOST_ERROR_CODE(socketStream.connect(hostResults, ec));

    // Send the request
    CHECK_BOOST_ERROR_CODE(http::write(socketStream, request, ec));

    // Receive the HTTP response
    flat_buffer buffer;
    http::response<http::string_body> response;
    http::read(socketStream, buffer, response, ec);
    if (ec.failed())
    {
        return false;
    }

    if (response.result() != http::status::ok)
    {
        return false;
    }
    responseOutput = response.body();
    return true;
}

bool Http::getHttps(const string& host, const string& port, const http::request<http::string_body>& request,
        string& responseOutput)
{
    boost::system::error_code ec;

    io_service ioService;

    // Host name resolution
    ip::tcp::resolver resolver(ioService);
    CHECK_BOOST_ERROR_CODE(auto hostResults = resolver.resolve({host, port}, ec));

    // Connection
    ssl::context sslContext(ssl::context::tlsv12_client);
    CHECK_BOOST_ERROR_CODE(loadRootCertificates(sslContext, ec));

    ssl::stream<ip::tcp::socket> socketStream(ioService, sslContext);
    if(!SSL_set_tlsext_host_name(socketStream.native_handle(), host.c_str()))
    {
        return false;
    }

    CHECK_BOOST_ERROR_CODE(boost::asio::connect(socketStream.lowest_layer(), hostResults, ec));
    CHECK_BOOST_ERROR_CODE(socketStream.handshake(ssl::stream_base::handshake_type::client, ec));

    // Send the request
    CHECK_BOOST_ERROR_CODE(http::write(socketStream, request, ec));

    // Receive the HTTP response
    flat_buffer buffer;
    http::response<http::string_body> response;
    CHECK_BOOST_ERROR_CODE(http::read(socketStream, buffer, response, ec));

    if (response.result() != http::status::ok)
    {
        return false;
    }
    responseOutput = response.body();
    return true;
}

void Http::loadRootCertificates(boost::asio::ssl::context& sslContext, boost::system::error_code& ec)
{
    // From https://www.boost.org/doc/libs/develop/libs/beast/example/common/root_certificates.hpp
    const string cert =
            /*  This is the DigiCert Global Root CA

                CN = DigiCert High Assurance EV Root CA
                OU = www.digicert.com
                O = DigiCert Inc
                C = US

                Valid to: 10 November 2031

                Serial #:
                08:3B:E0:56:90:42:46:B1:A1:75:6A:C9:59:91:C7:4A

                SHA1 Fingerprint:
                A8:98:5D:3A:65:E5:E5:C4:B2:D7:D6:6D:40:C6:DD:2F:B1:9C:54:36

                SHA256 Fingerprint:
                43:48:A0:E9:44:4C:78:CB:26:5E:05:8D:5E:89:44:B4:D8:4F:96:62:BD:26:DB:25:7F:89:34:A4:43:C7:01:61
            */
            "-----BEGIN CERTIFICATE-----\n"
            "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n"
            "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
            "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n"
            "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n"
            "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
            "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n"
            "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n"
            "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n"
            "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n"
            "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n"
            "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n"
            "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n"
            "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n"
            "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n"
            "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n"
            "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n"
            "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n"
            "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n"
            "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n"
            "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n"
            "-----END CERTIFICATE-----\n"

            /*  This is the GeoTrust root certificate.

                CN = GeoTrust Global CA
                O = GeoTrust Inc.
                C = US
                Valid to: Friday, ‎May ‎20, ‎2022 9:00:00 PM

                Thumbprint(sha1):
                ‎de 28 f4 a4 ff e5 b9 2f a3 c5 03 d1 a3 49 a7 f9 96 2a 82 12
            */
            "-----BEGIN CERTIFICATE-----\n"
            "MIIDaDCCAlCgAwIBAgIJAO8vBu8i8exWMA0GCSqGSIb3DQEBCwUAMEkxCzAJBgNV\n"
            "BAYTAlVTMQswCQYDVQQIDAJDQTEtMCsGA1UEBwwkTG9zIEFuZ2VsZXNPPUJlYXN0\n"
            "Q049d3d3LmV4YW1wbGUuY29tMB4XDTE3MDUwMzE4MzkxMloXDTQ0MDkxODE4Mzkx\n"
            "MlowSTELMAkGA1UEBhMCVVMxCzAJBgNVBAgMAkNBMS0wKwYDVQQHDCRMb3MgQW5n\n"
            "ZWxlc089QmVhc3RDTj13d3cuZXhhbXBsZS5jb20wggEiMA0GCSqGSIb3DQEBAQUA\n"
            "A4IBDwAwggEKAoIBAQDJ7BRKFO8fqmsEXw8v9YOVXyrQVsVbjSSGEs4Vzs4cJgcF\n"
            "xqGitbnLIrOgiJpRAPLy5MNcAXE1strVGfdEf7xMYSZ/4wOrxUyVw/Ltgsft8m7b\n"
            "Fu8TsCzO6XrxpnVtWk506YZ7ToTa5UjHfBi2+pWTxbpN12UhiZNUcrRsqTFW+6fO\n"
            "9d7xm5wlaZG8cMdg0cO1bhkz45JSl3wWKIES7t3EfKePZbNlQ5hPy7Pd5JTmdGBp\n"
            "yY8anC8u4LPbmgW0/U31PH0rRVfGcBbZsAoQw5Tc5dnb6N2GEIbq3ehSfdDHGnrv\n"
            "enu2tOK9Qx6GEzXh3sekZkxcgh+NlIxCNxu//Dk9AgMBAAGjUzBRMB0GA1UdDgQW\n"
            "BBTZh0N9Ne1OD7GBGJYz4PNESHuXezAfBgNVHSMEGDAWgBTZh0N9Ne1OD7GBGJYz\n"
            "4PNESHuXezAPBgNVHRMBAf8EBTADAQH/MA0GCSqGSIb3DQEBCwUAA4IBAQCmTJVT\n"
            "LH5Cru1vXtzb3N9dyolcVH82xFVwPewArchgq+CEkajOU9bnzCqvhM4CryBb4cUs\n"
            "gqXWp85hAh55uBOqXb2yyESEleMCJEiVTwm/m26FdONvEGptsiCmF5Gxi0YRtn8N\n"
            "V+KhrQaAyLrLdPYI7TrwAOisq2I1cD0mt+xgwuv/654Rl3IhOMx+fKWKJ9qLAiaE\n"
            "fQyshjlPP9mYVxWOxqctUdQ8UnsUKKGEUcVrA08i1OAnVKlPFjKBvk+r7jpsTPcr\n"
            "9pWXTO9JrYMML7d+XRSZA1n3856OqZDX4403+9FnXCvfcLZLLKTBvwwFgEFGpzjK\n"
            "UEVbkhd5qstF6qWK\n"
            "-----END CERTIFICATE-----\n";

    sslContext.add_certificate_authority(boost::asio::buffer(cert.data(), cert.size()), ec);
}
