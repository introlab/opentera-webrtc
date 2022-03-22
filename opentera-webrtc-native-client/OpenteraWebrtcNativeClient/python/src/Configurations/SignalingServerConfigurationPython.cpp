#include <OpenteraWebrtcNativeClientPython/Configurations/SignalingServerConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/SioMessage.h>

#include <OpenteraWebrtcNativeClient/Configurations/SignalingServerConfiguration.h>

#include <pybind11/stl.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

SignalingServerConfiguration create(string url, string clientName, const py::object& clientData, string room)
{
    return SignalingServerConfiguration::create(
        move(url),
        move(clientName),
        pyObjectToSioMessage(clientData),
        move(room));
}

SignalingServerConfiguration
    create(string url, string clientName, const py::object& clientData, string room, string password)
{
    return SignalingServerConfiguration::create(
        move(url),
        move(clientName),
        pyObjectToSioMessage(clientData),
        move(room),
        move(password));
}

void opentera::initSignalingServerConfigurationPython(pybind11::module& m)
{
    py::class_<SignalingServerConfiguration>(
        m,
        "SignalingServerConfiguration",
        "Represents a signaling server configuration.")
        .def_static(
            "create",
            py::overload_cast<string, string, string>(&SignalingServerConfiguration::create),
            "Creates an signaling server configuration with the specified "
            "values.\n"
            "\n"
            ":param url: The signaling server URL\n"
            ":param client_name: The client name\n"
            ":param room: The room name\n"
            ":return: A signaling server configuration with the specified values",
            py::arg("url"),
            py::arg("client_name"),
            py::arg("room"))
        .def_static(
            "create",
            py::overload_cast<string, string, const py::object&, string>(&create),
            "Creates an signaling server configuration with the specified "
            "values.\n"
            "     *\n"
            ":param url: The signaling server URL\n"
            ":param client_name: The client name\n"
            ":param client_data: The client data\n"
            ":param room: The room name\n"
            ":return: A signaling server configuration with the specified values",
            py::arg("url"),
            py::arg("client_name"),
            py::arg("client_data"),
            py::arg("room"))
        .def_static(
            "create",
            py::overload_cast<string, string, string, string>(&SignalingServerConfiguration::create),
            "Creates an signaling server configuration with the specified "
            "values.\n"
            "\n"
            ":param url: The signaling server URL\n"
            ":param client_name: The client name\n"
            ":param room: The room name\n"
            ":param password: The signaling server password\n"
            ":return: A signaling server configuration with the specified values",
            py::arg("url"),
            py::arg("client_name"),
            py::arg("room"),
            py::arg("password"))
        .def_static(
            "create",
            py::overload_cast<string, string, const py::object&, string, string>(&create),
            "Creates an signaling server configuration with the specified "
            "values.\n"
            "\n"
            ":param url: The signaling server URL\n"
            ":param client_name: The client name\n"
            ":param client_data: The client data\n"
            ":param room: The room name\n"
            ":param password: The signaling server password\n"
            ":return: A signaling server configuration with the specified values",
            py::arg("url"),
            py::arg("client_name"),
            py::arg("client_data"),
            py::arg("room"),
            py::arg("password"))

        .def_property_readonly(
            "url",
            &SignalingServerConfiguration::url,
            "Returns the signaling server URL.\n"
            ":return: The signaling server URL")
        .def_property_readonly(
            "client_name",
            &SignalingServerConfiguration::clientName,
            "Returns the client name.\n"
            ":return: The client name")
        .def_property_readonly(
            "client_data",
            [](const SignalingServerConfiguration& self) { return sioMessageToPyObject(self.clientData()); },
            "Returns the client data.\n"
            ":return: The client data")
        .def_property_readonly(
            "room",
            &SignalingServerConfiguration::room,
            "Returns the room name.\n"
            ":return: The room name")
        .def_property_readonly(
            "password",
            &SignalingServerConfiguration::password,
            "Returns the signaling server password.\n"
            ":return: The signaling server password");
}
