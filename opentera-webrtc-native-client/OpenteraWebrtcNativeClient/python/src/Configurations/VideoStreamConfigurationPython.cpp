#include <OpenteraWebrtcNativeClientPython/Configurations/VideoStreamConfigurationPython.h>

#include <OpenteraWebrtcNativeClient/Configurations/VideoStreamConfiguration.h>

#include <pybind11/stl.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initVideoStreamConfigurationPython(py::module& m)
{
    py::enum_<VideoStreamCodec>(m, "VideoStreamCodec")
        .value("VP8", VideoStreamCodec::VP8)
        .value("VP9", VideoStreamCodec::VP9)
        .value("H264", VideoStreamCodec::H264);

    py::class_<VideoStreamConfiguration>(m, "VideoStreamConfiguration", "Represents a video stream configuration")
        .def_static(
            "create",
            py::overload_cast<>(&VideoStreamConfiguration::create),
            "Creates a stream configuration with default values.\n"
            "\n"
            ":return: A stream configuration with default values")
        .def_static(
            "create",
            py::overload_cast<unordered_set<VideoStreamCodec>>(&VideoStreamConfiguration::create),
            "Creates a video stream configuration with the specified value.\n"
            "\n"
            ":param forced_codecs: Indicates the codecs that must be used. An empty set means all codecs.\n"
            "\n"
            ":return: A video stream configuration with the specified value",
            py::arg("forced_codecs"))
        .def_static(
            "create",
            py::overload_cast<unordered_set<VideoStreamCodec>, bool, bool>(&VideoStreamConfiguration::create),
            "Creates a video stream configuration with the specified values.\n"
            "\n"
            ":param forced_codecs: Indicates the codecs that must be used. An empty set means all codecs.\n"
            ":param force_gstreamer_hardware_acceleration: Indicates that hardware accelerated codecs must be used. It "
            "has no effect when the library is not built with GStreamer.\n"
            ":param use_gstreamer_software_encoder_decoder: Indicates to use GStreamer software codecs instead of "
            "WebRTC ones. It has no effect when the library is not built with GStreamer.\n"
            "\n"
            ":return: A video stream configuration with the specified values",
            py::arg("forced_codecs"),
            py::arg("force_gstreamer_hardware_acceleration"),
            py::arg("use_gstreamer_software_encoder_decoder"))

        .def_property_readonly(
            "forced_codecs",
            &VideoStreamConfiguration::forcedCodecs,
            "Returns the codecs that must be used. An empty set means all codecs.\n"
            "\n"
            ":return: The codecs that must be used")
        .def_property_readonly(
            "force_gstreamer_hardware_acceleration",
            &VideoStreamConfiguration::forceGStreamerHardwareAcceleration,
            "Indicates that hardware accelerated codecs must be used.\n"
            "\n"
            ":return: True if only hardware accelerated codecs can be used.")
        .def_property_readonly(
            "use_gstreamer_software_encoder_decoder",
            &VideoStreamConfiguration::useGStreamerSoftwareEncoderDecoder,
            "Indicates to use GStreamer software codecs instead of WebRTC ones.\n"
            "\n"
            ":return: True if GStreamer software codecs must be used instead of WebRTC ones");
}
