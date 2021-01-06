#include <OpenteraWebrtcNativeClientPython/Configurations/SignallingServerConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/SioMessage.h>

#include <OpenteraWebrtcNativeClient/Configurations/SignallingServerConfiguration.h>

#include <pybind11/stl.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

SignallingServerConfiguration create(string url, string clientName, const py::object& clientData, string room)
{
    return SignallingServerConfiguration::create(move(url), move(clientName), pyObjectToSioMessage(clientData),
            move(room));
}

SignallingServerConfiguration create(string url, string clientName, const py::object& clientData, string room,
        string password)
{
    return SignallingServerConfiguration::create(move(url), move(clientName), pyObjectToSioMessage(clientData),
            move(room), move(password));
}

void opentera::initSignallingServerConfigurationPython(pybind11::module &m)
{
    py::class_<SignallingServerConfiguration>(m, "SignallingServerConfiguration")
            .def_static("create",
                    py::overload_cast<string, string, string>(&SignallingServerConfiguration::create),
                    py::arg("url"), py::arg("client_name"), py::arg("room"))
            .def_static("create",
                    py::overload_cast<string, string, const py::object&, string>(&create),
                    py::arg("url"), py::arg("client_name"), py::arg("client_data"), py::arg("room"))
            .def_static("create",
                    py::overload_cast<string, string, string, string>(&SignallingServerConfiguration::create),
                    py::arg("url"), py::arg("client_name"), py::arg("room"), py::arg("password"))
            .def_static("create",
                    py::overload_cast<string, string, const py::object&, string, string>(&create),
                    py::arg("url"), py::arg("client_name"), py::arg("client_data"), py::arg("room"), py::arg("password"))

            .def_property_readonly("url", &SignallingServerConfiguration::url)
            .def_property_readonly("client_name", &SignallingServerConfiguration::clientName)
            .def_property_readonly("client_data",
                    [](const SignallingServerConfiguration& self) { return sioMessageToPyObject(self.clientData()); })
            .def_property_readonly("room", &SignallingServerConfiguration::room)
            .def_property_readonly("password", &SignallingServerConfiguration::password);
}
