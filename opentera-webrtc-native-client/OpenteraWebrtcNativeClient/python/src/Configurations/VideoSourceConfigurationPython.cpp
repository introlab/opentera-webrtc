#include <OpenteraWebrtcNativeClientPython/Configurations/VideoSourceConfigurationPython.h>

#include <OpenteraWebrtcNativeClient/Configurations/VideoSourceConfiguration.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initVideoSourceConfigurationPython(py::module& m)
{
    py::class_<VideoSourceConfiguration>(m, "VideoSourceConfiguration")
            .def_static("create", py::overload_cast<bool, bool>(&VideoSourceConfiguration::create),
                    py::arg("needs_denoising"), py::arg("is_screencast"))

            .def_property_readonly("needs_denoising", &VideoSourceConfiguration::needsDenoising)
            .def_property_readonly("is_screencast", &VideoSourceConfiguration::isScreencast);
}
