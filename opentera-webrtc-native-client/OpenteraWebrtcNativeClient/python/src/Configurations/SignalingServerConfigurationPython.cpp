#include <OpenteraWebrtcNativeClientPython/Configurations/SignalingServerConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/SioMessage.h>

#include <OpenteraWebrtcNativeClient/Configurations/SignalingServerConfiguration.h>

#include <pybind11/stl.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

SignalingServerConfiguration create(string url, string clientName, const py::object& clientData, string room)
{
    return SignalingServerConfiguration::create(move(url), move(clientName), pyObjectToSioMessage(clientData),
                                                move(room));
}

SignalingServerConfiguration create(string url, string clientName, const py::object& clientData, string room,
                                    string password)
{
    return SignalingServerConfiguration::create(move(url), move(clientName), pyObjectToSioMessage(clientData),
                                                move(room), move(password));
}

void opentera::initSignalingServerConfigurationPython(pybind11::module &m)
{
    py::class_<SignalingServerConfiguration>(m, "SignalingServerConfiguration")
            .def_static("create",
                    py::overload_cast<string, string, string>(&SignalingServerConfiguration::create),
                    py::arg("url"), py::arg("client_name"), py::arg("room"))
            .def_static("create",
                    py::overload_cast<string, string, const py::object&, string>(&create),
                    py::arg("url"), py::arg("client_name"), py::arg("client_data"), py::arg("room"))
            .def_static("create",
                    py::overload_cast<string, string, string, string>(&SignalingServerConfiguration::create),
                    py::arg("url"), py::arg("client_name"), py::arg("room"), py::arg("password"))
            .def_static("create",
                    py::overload_cast<string, string, const py::object&, string, string>(&create),
                    py::arg("url"), py::arg("client_name"), py::arg("client_data"), py::arg("room"), py::arg("password"))

            .def_property_readonly("url", &SignalingServerConfiguration::url)
            .def_property_readonly("client_name", &SignalingServerConfiguration::clientName)
            .def_property_readonly("client_data",
                    [](const SignalingServerConfiguration& self) { return sioMessageToPyObject(self.clientData()); })
            .def_property_readonly("room", &SignalingServerConfiguration::room)
            .def_property_readonly("password", &SignalingServerConfiguration::password);
}
