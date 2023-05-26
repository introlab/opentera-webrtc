#include <OpenteraWebrtcNativeClientPython/Utils/IceServerPython.h>

#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

#include <pybind11/stl.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initIceServerPython(pybind11::module& m)
{
    py::class_<IceServer>(m, "IceServer")
        .def(
            py::init<string>(),
            "Creates an ice server configuration with the specified value.\n"
            "\n"
            ":param url: The ice server url",
            py::arg("url"))
        .def(
            py::init<string, string, string>(),
            "Creates an ice server configuration with the specified values.\n"
            "\n"
            ":param url: The ice server url\n"
            ":param username: The ice server username\n"
            ":param credential: The ice server credential",
            py::arg("url"),
            py::arg("username"),
            py::arg("credential"))
        .def(
            py::init<vector<string>>(),
            "Creates an ice server configuration with the specified value.\n"
            "\n"
            ":param urls: The ice server urls",
            py::arg("urls"))
        .def(
            py::init<vector<string>, string, string>(),
            "Creates an ice server configuration with the specified values.\n"
            "\n"
            ":param urls: The ice server urls\n"
            ":param username: The ice server username\n"
            ":param credential: The ice server credential",
            py::arg("urls"),
            py::arg("username"),
            py::arg("credential"))

        .def_property_readonly(
            "urls",
            &IceServer::urls,
            "Returns the ice server urls.\n"
            "\n"
            ":return: The ice server urls")
        .def_property_readonly(
            "username",
            &IceServer::username,
            "Returns the ice server username.\n"
            "\n"
            ":return: The ice server username")
        .def_property_readonly(
            "credential",
            &IceServer::credential,
            "Returns the ice server credential.\n"
            "\n"
            ":return: The ice server credential")

        .def_static(
            "fetch_from_server",
            [](const string& url, const string& password)
            {
                vector<IceServer> iceServers;
                if (IceServer::fetchFromServer(url, password, iceServers))
                {
                    return iceServers;
                }
                else
                {
                    throw runtime_error("\"fetch_from_server\" failed");
                }
            },
            "Fetches the ice servers from the signaling server.\n"
            "\n"
            ":param url: The signaling server url\n"
            ":param password: The signaling server username\n"
            "\n"
            ":return: The fetched ice servers",
            py::arg("url"),
            py::arg("password"))
        .def_static(
            "fetch_from_server",
            [](const string& url, const string& password, bool verifyCertificate)
            {
                vector<IceServer> iceServers;
                if (IceServer::fetchFromServer(url, password, iceServers, verifyCertificate))
                {
                    return iceServers;
                }
                else
                {
                    throw runtime_error("\"fetch_from_server\" failed");
                }
            },
            "Fetches the ice servers from the signaling server.\n"
            "\n"
            ":param url: The signaling server url\n"
            ":param password: The signaling server username\n"
            ":param verify_certificate: Indicates to verify the certificate or "
            "not\n"
            "\n"
            ":return: The fetched ice servers",
            py::arg("url"),
            py::arg("password"),
            py::arg("verify_certificate"))
        .def_static(
            "from_json",
            [](const string& json)
            {
                vector<IceServer> iceServers;
                if (IceServer::fromJson(json, iceServers))
                {
                    return iceServers;
                }
                else
                {
                    throw py::value_error("Invalid json");
                }
            },
            "Gets ice servers from a JSON\n"
            "\n"
            ":param json: The JSON to parse\n"
            "\n"
            ":return The parsed ice servers",
            py::arg("json"));
}
