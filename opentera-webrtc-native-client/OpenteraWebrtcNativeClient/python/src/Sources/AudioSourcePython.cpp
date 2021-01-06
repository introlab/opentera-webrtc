#include <OpenteraWebrtcNativeClientPython/Sources/AudioSourcePython.h>

#include <OpenteraWebrtcNativeClient/Sources/AudioSource.h>

#include <pybind11/numpy.h>

using namespace introlab;
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

void introlab::initAudioSourcePython(pybind11::module& m)
{
    py::class_<AudioSource, shared_ptr<AudioSource>>(m, "AudioSource")
        .def(py::init<AudioSourceConfiguration, int, int, size_t>(),
                py::arg("configuration"), py::arg("bits_per_sample"), py::arg("sample_rate"), py::arg("number_of_channels"))
        .def("send_frame", &sendFrame<int8_t>, py::arg("frame"))
        .def("send_frame", &sendFrame<int16_t>, py::arg("frame"))
        .def("send_frame", &sendFrame<int32_t>, py::arg("frame"));
}
