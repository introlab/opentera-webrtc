#include <OpenteraWebrtcNativeClientPython/Configurations/WebrtcConfigurationPython.h>

#include <OpenteraWebrtcNativeClient/Configurations/WebrtcConfiguration.h>

#include <pybind11/stl.h>

using namespace introlab;
using namespace std;
namespace py = pybind11;

void introlab::initWebrtcConfigurationPython(pybind11::module &m)
{
    py::class_<WebrtcConfiguration>(m, "WebrtcConfiguration")
            .def_static("create", py::overload_cast<>(&WebrtcConfiguration::create))
            .def_static("create", py::overload_cast<vector<IceServer>>(&WebrtcConfiguration::create),
                    py::arg("ice_servers"))

            .def_property_readonly("ice_servers", &WebrtcConfiguration::iceServers);
}
