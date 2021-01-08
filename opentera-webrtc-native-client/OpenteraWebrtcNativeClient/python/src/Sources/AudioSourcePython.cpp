#include <OpenteraWebrtcNativeClientPython/Sources/AudioSourcePython.h>

#include <OpenteraWebrtcNativeClient/Sources/AudioSource.h>

#include <pybind11/numpy.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

template <class T>
void sendFrame(const shared_ptr<AudioSource>& self, py::array_t<T>& frame)
{
    if (self->bytesPerSample() != sizeof(T))
    {
        throw py::value_error("Invalid frame data type.");
    }
    if (frame.ndim() != 1)
    {
        throw py::value_error("The frame must have 1 dimension.");
    }

    size_t byteSize = sizeof(T) * frame.shape(0);
    if (byteSize % self->bytesPerFrame() != 0)
    {
        throw py::value_error("The frame size must be a multiple of (bytes_per_sample * number_of_channels).");
    }

    self->sendFrame(frame.data(), byteSize / self->bytesPerFrame());
}

void opentera::initAudioSourcePython(pybind11::module& m)
{
    py::class_<AudioSource, shared_ptr<AudioSource>>(m, "AudioSource",
            "Represent a audio source that can be added to a WebRTC call.\n"
            "\n"
            "Pass an instance of this to the StreamClient and call sendFrame for each of your audio frame.")
            .def(py::init<AudioSourceConfiguration, int, int, size_t>(),
                    "Creates an AudioSource\n"
                    "\n"
                    ":param configuration: the configuration applied to the audio stream by the audio transport layer\n"
                    ":param bits_per_sample: The audio stream sample size (8, 16 or 32 bits)\n"
                    ":param sample_rate: The audio stream sample rate\n"
                    ":param number_of_channels: The audio stream channel count",
                    py::arg("configuration"), py::arg("bits_per_sample"), py::arg("sample_rate"), py::arg("number_of_channels"))
            .def("send_frame", &sendFrame<int8_t>,
                    "Send an audio frame\n"
                    ":param frame: The audio frame",
                    py::arg("frame"))
            .def("send_frame", &sendFrame<int16_t>,
                     "Send an audio frame\n"
                     ":param frame: The audio frame",
                     py::arg("frame"))
            .def("send_frame", &sendFrame<int32_t>,
                     "Send an audio frame\n"
                     ":param frame: The audio frame",
                     py::arg("frame"));
}
