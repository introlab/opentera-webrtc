#include <OpenteraWebrtcNativeClientPython/Configurations/AudioSourceConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/PyBindAbslOptional.h>

#include <OpenteraWebrtcNativeClient/Configurations/AudioSourceConfiguration.h>

using namespace introlab;
using namespace std;
namespace py = pybind11;

void introlab::initAudioSourceConfigurationPython(py::module& m)
{
    py::class_<AudioSourceConfiguration>(m, "AudioSourceConfiguration")
            .def_static("create", py::overload_cast<>(&AudioSourceConfiguration::create))
            .def_static("create", py::overload_cast<absl::optional<bool>, absl::optional<bool>, absl::optional<bool>, absl::optional<bool>, absl::optional<bool>, absl::optional<bool>, absl::optional<bool>>(&AudioSourceConfiguration::create),
                    py::arg("echo_cancellation"), py::arg("auto_gain_control"), py::arg("noise_suppression"), py::arg("highpass_filter"), py::arg("stereo_swapping"), py::arg("typing_detection"), py::arg("residual_echo_detector"))

            .def_property_readonly("echo_cancellation", &AudioSourceConfiguration::echoCancellation)
            .def_property_readonly("auto_gain_control", &AudioSourceConfiguration::autoGainControl)
            .def_property_readonly("noise_suppression", &AudioSourceConfiguration::noiseSuppression)
            .def_property_readonly("highpass_filter", &AudioSourceConfiguration::highpassFilter)
            .def_property_readonly("stereo_swapping", &AudioSourceConfiguration::stereoSwapping)
            .def_property_readonly("typing_detection", &AudioSourceConfiguration::typingDetection)
            .def_property_readonly("residual_echo_detector", &AudioSourceConfiguration::residualEchoDetector);
}
