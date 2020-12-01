#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_HTTP_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_HTTP_H

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/beast/version.hpp>

#include <cstdint>
#include <functional>
#include <string>

namespace introlab
{
    class Http
    {
    public:
        static bool get(const std::string& url, std::string& response,
                std::function<void(boost::beast::http::request<boost::beast::http::string_body>&)> requestModifier);
        static bool getHostPortTargetFromUrl(const std::string& url, const std::string& defaultPort,
                std::string& host, std::string& port, std::string& endpoint);

    private:
        static bool getHttp(const std::string& host, const std::string& port,
                const boost::beast::http::request<boost::beast::http::string_body>& request,
                std::string& responseOutput);
        static bool getHttps(const std::string& host, const std::string& port,
                const boost::beast::http::request<boost::beast::http::string_body>& request,
                std::string& responseOutput);

        static void loadRootCertificates(boost::asio::ssl::context& sslContext, boost::system::error_code& ec);
    };
}

#endif
