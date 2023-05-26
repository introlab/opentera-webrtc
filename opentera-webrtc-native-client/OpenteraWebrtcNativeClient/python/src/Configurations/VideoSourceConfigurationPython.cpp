#include <OpenteraWebrtcNativeClientPython/Configurations/VideoSourceConfigurationPython.h>

#include <OpenteraWebrtcNativeClient/Configurations/VideoSourceConfiguration.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initVideoSourceConfigurationPython(py::module& m)
{
    py::class_<VideoSourceConfiguration>(
        m,
        "VideoSourceConfiguration",
        "Represents a configuration of a video source that can be added to a "
        "WebRTC call.")
        .def_static(
            "create",
            py::overload_cast<bool, bool>(&VideoSourceConfiguration::create),
            "Creates a video source configuration with the specified values.\n"
            "\n"
            ":param needs_denoising: Indicates if this source needs denoising\n"
            ":param is_screencast: Indicates if this source is screencast\n"
            "\n"
            ":return: A video source configuration with the specified values",
            py::arg("needs_denoising"),
            py::arg("is_screencast"))

        .def_property_readonly(
            "needs_denoising",
            &VideoSourceConfiguration::needsDenoising,
            "Indicates if this source needs denoising.\n"
            "\n"
            ":return: True if this source needs denoising")
        .def_property_readonly(
            "is_screencast",
            &VideoSourceConfiguration::isScreencast,
            "Indicates if this source is screencast.\n"
            "\n"
            ":return: True if this source is a screencast");
}
