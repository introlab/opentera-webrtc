#include <OpenteraWebrtcNativeClientPython/StreamClientPython.h>

#include <OpenteraWebrtcNativeClient/StreamClient.h>

using namespace introlab;
using namespace std;
namespace py = pybind11;


void introlab::initStreamClientPython(pybind11::module& m)
{
    py::class_<StreamClient, SignallingClient>(m, "StreamClient")
        .def(py::init<const SignallingServerConfiguration&, const WebrtcConfiguration&, const std::shared_ptr<VideoSource>&>(),
             py::arg("signalling_server_configuration"), py::arg("webrtc_configuration"), py::arg("video_source"));
}
