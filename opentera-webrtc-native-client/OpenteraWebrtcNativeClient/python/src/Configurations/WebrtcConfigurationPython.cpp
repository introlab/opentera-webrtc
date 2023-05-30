#include <OpenteraWebrtcNativeClientPython/Configurations/WebrtcConfigurationPython.h>

#include <OpenteraWebrtcNativeClient/Configurations/WebrtcConfiguration.h>

#include <pybind11/stl.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initWebrtcConfigurationPython(pybind11::module& m)
{
    py::class_<WebrtcConfiguration>(m, "WebrtcConfiguration", "Represents a WebRTC peer connection configuration.")
        .def_static(
            "create",
            py::overload_cast<>(&WebrtcConfiguration::create),
            "Creates a WebRTC peer connection configuration with default "
            "values.\n"
            "\n"
            ":return: A WebRTC peer connection configuration with default values")
        .def_static(
            "create",
            py::overload_cast<vector<IceServer>>(&WebrtcConfiguration::create),
            "Creates a WebRTC peer connection configuration with the specified "
            "value.\n"
            "\n"
            ":param ice_servers: The ice servers\n"
            "\n"
            ":return: A WebRTC peer connection configuration with the specified "
            "value",
            py::arg("ice_servers"))

        .def_property_readonly(
            "ice_servers",
            &WebrtcConfiguration::iceServers,
            "Returns the ice servers.\n"
            "\n"
            ":return: The ice servers");
}
