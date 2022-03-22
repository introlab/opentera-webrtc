#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

#include <OpenteraWebrtcNativeClient/Utils/Http.h>

#include <rapidjson/document.h>

using namespace opentera;
using namespace rapidjson;
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

bool parseIceServerJson(GenericValue<UTF8<>>& iceServerJson, vector<IceServer>& iceServers)
{
    if (!iceServerJson.IsObject() || !iceServerJson.HasMember("urls"))
    {
        return false;
    }

    vector<string> urls;
    auto& urlsJson = iceServerJson["urls"];
    if (urlsJson.IsString())
    {
        urls.emplace_back(urlsJson.GetString());
    }
    else if (urlsJson.IsArray())
    {
        for (auto it = urlsJson.Begin(); it != urlsJson.End(); ++it)
        {
            if (!it->IsString())
            {
                return false;
            }
            urls.emplace_back(it->GetString());
        }
    }
    else
    {
        return false;
    }

    if (iceServerJson.HasMember("username") && iceServerJson.HasMember("credential"))
    {
        auto& usernameJson = iceServerJson["username"];
        auto& credentialJson = iceServerJson["credential"];
        if (!usernameJson.IsString() || !credentialJson.IsString())
        {
            return false;
        }
        iceServers.emplace_back(IceServer(urls, usernameJson.GetString(), credentialJson.GetString()));
    }
    else if (!iceServerJson.HasMember("username") && !iceServerJson.HasMember("credential"))
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
    Document jsonDocument;
    jsonDocument.Parse(json.data());

    if (!jsonDocument.IsArray())
    {
        return false;
    }

    for (auto it = jsonDocument.Begin(); it != jsonDocument.End(); ++it)
    {
        if (!parseIceServerJson(*it, iceServers))
        {
            return false;
        }
    }
    return true;
}
