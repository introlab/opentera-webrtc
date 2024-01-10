#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

#include <OpenteraWebrtcNativeClient/Utils/Http.h>

#include <nlohmann/json.hpp>

using namespace opentera;
using namespace std;

/**
 * @brief Creates an ice server configuration with the specified value.
 *
 * @param url The ice server url
 */
IceServer::IceServer(string url) : m_urls({move(url)}) {}

/**
 * @brief Creates an ice server configuration with the specified values.
 *
 * @param url The ice server url
 * @param username The ice server username
 * @param credential The ice server credential
 */
IceServer::IceServer(string url, string username, string credential)
    : m_urls({move(url)}),
      m_username(move(username)),
      m_credential(move(credential))
{
}

/**
 * @brief Creates an ice server configuration with the specified value.
 *
 * @param urls The ice server urls
 */
IceServer::IceServer(vector<string> urls) : m_urls(move(urls)) {}

/**
 * @brief Creates an ice server configuration with the specified values.
 *
 * @param urls The ice server urls
 * @param username The ice server username
 * @param credential The ice server credential
 */
IceServer::IceServer(vector<string> urls, string username, string credential)
    : m_urls(move(urls)),
      m_username(move(username)),
      m_credential(move(credential))
{
}

/**
 * @brief Fetches the ice servers from the signaling server.
 *
 * @param url The signaling server url
 * @param password The signaling server username
 * @param iceServers The fetched ice servers
 * @param verifyCertificate Indicates to verify the certificate or not
 * @return true if success
 */
bool IceServer::fetchFromServer(
    const string& url,
    const string& password,
    vector<IceServer>& iceServers,
    bool verifyCertificate)
{
    string response;
    if (!Http::get(url, response, {{"Authorization", password.c_str()}}, verifyCertificate))
    {
        return false;
    }

    return fromJson(response, iceServers);
}

bool parseIceServerJson(const nlohmann::json& iceServerJson, vector<IceServer>& iceServers)
{
    if (!iceServerJson.is_object() || !iceServerJson.contains("urls"))
    {
        return false;
    }

    vector<string> urls;
    auto& urlsJson = iceServerJson["urls"];
    if (urlsJson.is_string())
    {
        urls.emplace_back(urlsJson);
    }
    else if (urlsJson.is_array())
    {
        for (auto& it : urlsJson)
        {
            if (!it.is_string())
            {
                return false;
            }
            urls.emplace_back(it);
        }
    }
    else
    {
        return false;
    }

    if (iceServerJson.contains("username") && iceServerJson.contains("credential"))
    {
        auto& usernameJson = iceServerJson["username"];
        auto& credentialJson = iceServerJson["credential"];
        if (!usernameJson.is_string() || !credentialJson.is_string())
        {
            return false;
        }
        iceServers.emplace_back(IceServer(urls, usernameJson, credentialJson));
    }
    else if (!iceServerJson.contains("username") && !iceServerJson.contains("credential"))
    {
        iceServers.emplace_back(IceServer(urls));
    }
    else
    {
        return false;
    }
    return true;
}

/**
 * @brief Gets ice servers from a JSON
 *
 * @param json The JSON to parse
 * @param iceServers The parsed ice servers
 * @return true if success
 */
bool IceServer::fromJson(const string& json, vector<IceServer>& iceServers)
{
    nlohmann::json parsedJson = nlohmann::json::parse(json, nullptr, false);

    if (parsedJson.is_discarded() || !parsedJson.is_array())
    {
        return false;
    }

    for (auto& it : parsedJson)
    {
        if (!parseIceServerJson(it, iceServers))
        {
            return false;
        }
    }
    return true;
}
