#include <OpenteraWebrtcNativeClientPython/Configurations/SignallingServerConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/SioMessage.h>

#include <OpenteraWebrtcNativeClient/Configurations/SignallingServerConfiguration.h>

#include <pybind11/stl.h>

using namespace introlab;
using namespace std;
namespace py = pybind11;

SignallingServerConfiguration create(const string& url, const string& clientName, py::object clientData,
        const string& room)
{
    return SignallingServerConfiguration::create(url, clientName, pyObjectToSioMessage(clientData), room);
}

SignallingServerConfiguration create(const string& url, const string& clientName, py::object clientData,
                                     const string& room, const string& password)
{
    return SignallingServerConfiguration::create(url, clientName, pyObjectToSioMessage(clientData), room, password);
}

void introlab::initSignallingServerConfigurationPython(pybind11::module &m)
{
    py::class_<SignallingServerConfiguration>(m, "SignallingServerConfiguration")
            .def_static("create",
                    py::overload_cast<const string&, const string&, const string&>(&SignallingServerConfiguration::create),
                    py::arg("url"), py::arg("client_name"), py::arg("room"))
            .def_static("create",
                    py::overload_cast<const string&, const string&, py::object, const string&>(&create),
                    py::arg("url"), py::arg("client_name"), py::arg("client_data"), py::arg("room"))
            .def_static("create",
                    py::overload_cast<const string&, const string&, const string&, const string&>(&SignallingServerConfiguration::create),
                    py::arg("url"), py::arg("client_name"), py::arg("room"), py::arg("password"))
            .def_static("create",
                    py::overload_cast<const string&, const string&, py::object, const string&, const string&>(&create),
                    py::arg("url"), py::arg("client_name"), py::arg("client_data"), py::arg("room"), py::arg("password"))

            .def_property_readonly("url", &SignallingServerConfiguration::url)
            .def_property_readonly("client_name", &SignallingServerConfiguration::clientName)
            .def_property_readonly("client_data",
                    [](const SignallingServerConfiguration& self) { return sioMessageToPyObject(self.clientData()); })
            .def_property_readonly("room", &SignallingServerConfiguration::room)
            .def_property_readonly("password", &SignallingServerConfiguration::password);
}
