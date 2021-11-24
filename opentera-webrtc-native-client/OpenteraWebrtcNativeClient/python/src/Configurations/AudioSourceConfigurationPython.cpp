#include <OpenteraWebrtcNativeClientPython/Configurations/AudioSourceConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/PyBindAbslOptional.h>

#include <OpenteraWebrtcNativeClient/Configurations/AudioSourceConfiguration.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initAudioSourceConfigurationPython(py::module& m)
{
    py::class_<AudioSourceConfiguration>(m, "AudioSourceConfiguration",
            "Represents a configuration of an audio source that can be added to a WebRTC call")
            .def_static("create", py::overload_cast<uint32_t>(&AudioSourceConfiguration::create),
                    "Creates an audio source configuration with default values.\n"
                    ":param sound_card_total_delay_ms: The sum of the playback and recording delays\n"
                    ":return: An audio source configuration with default values",
                    py::arg("sound_card_total_delay_ms"))
            .def_static("create", py::overload_cast<uint32_t, absl::optional<bool>, absl::optional<bool>, absl::optional<bool>, absl::optional<bool>, absl::optional<bool>, absl::optional<bool>, absl::optional<bool>, absl::optional<bool>>(&AudioSourceConfiguration::create),
                    "Creates an audio source configuration with the specified values.\n"
                    "\n"
                    ":param sound_card_total_delay_ms: The sum of the playback and recording delays\n"
                    ":param echo_cancellation: Enable or disable the echo cancellation\n"
                    ":param auto_gain_control: Enable or disable the auto gain control\n"
                    ":param noise_suppression: Enable or disable the noise suppression\n"
                    ":param highpass_filter: Enable or disable the highpass filter\n"
                    ":param stereo_swapping: Enable or disable the stereo swapping\n"
                    ":param typing_detection: Enable or disable the typing detection\n"
                    ":param residual_echo_detector: Enable or disable the residual echo detector\n"
                    ":param transient_suppression: Enable or disable the transient suppression\n"
                    ":return: An audio source configuration with the specified values",
                    py::arg("sound_card_total_delay_ms"), py::arg("echo_cancellation"), py::arg("auto_gain_control"), py::arg("noise_suppression"), py::arg("highpass_filter"), py::arg("stereo_swapping"), py::arg("typing_detection"), py::arg("residual_echo_detector"), py::arg("transient_suppression"))

            .def_property_readonly("sound_card_total_delay_ms", &AudioSourceConfiguration::soundCardTotalDelayMs,
                    "Returns the sum of the playback and recording delays.\n"
                    ":return: The sum of the playback and recording delays")
            .def_property_readonly("echo_cancellation", &AudioSourceConfiguration::echoCancellation,
                    "Indicates if the echo cancellation is enabled.\n"
                    ":return: True if the echo cancellation is enabled")
            .def_property_readonly("auto_gain_control", &AudioSourceConfiguration::autoGainControl,
                    "Indicates if the auto gain control is enabled.\n"
                    ":return: True if the auto gain control is enabled")
            .def_property_readonly("noise_suppression", &AudioSourceConfiguration::noiseSuppression,
                    "Indicates if the noise suppression is enabled.\n"
                    ":return: True if the noise suppression is enabled")
            .def_property_readonly("highpass_filter", &AudioSourceConfiguration::highpassFilter,
                    "Indicates if the highpass filter is enabled.\n"
                    ":return: True if the highpass filter is enabled")
            .def_property_readonly("stereo_swapping", &AudioSourceConfiguration::stereoSwapping,
                    "Indicates if the stereo swapping is enabled.\n"
                    ":return: true if the stereo swapping is enabled")
            .def_property_readonly("typing_detection", &AudioSourceConfiguration::typingDetection,
                    "Indicates if the typing detection is enabled.\n"
                    ":return: True if the typing detection is enabled")
            .def_property_readonly("residual_echo_detector", &AudioSourceConfiguration::residualEchoDetector,
                    "Indicates if the residual echo detector is enabled.\n"
                    ":return: True if the residual echo detector is enabled")
            .def_property_readonly("transient_suppression", &AudioSourceConfiguration::transientSuppression,
                    "Indicates if the transient suppression is enabled.\n"
                    ":return: True if the transient suppression is enabled");
}
