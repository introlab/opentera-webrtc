#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_HTTP_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_HTTP_H

#include <cstdint>
#include <functional>
#include <string>
#include <map>

namespace opentera
{
    class Http
    {
    public:
        static bool get(const std::string& url, std::string& response, std::multimap<std::string, std::string> headers);
        static bool splitUrl(const std::string& url, std::string& host, std::string& target);
    };
}

#endif
