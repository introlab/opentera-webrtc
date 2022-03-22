#include <OpenteraWebrtcNativeClientPython/Sources/VideoSourcePython.h>

#include <OpenteraWebrtcNativeClient/Sources/VideoSource.h>

#include <pybind11/numpy.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void sendFrame(const shared_ptr<VideoSource>& self, const py::array_t<uint8_t>& bgrImg, int64_t timestampUs)
{
    if (bgrImg.ndim() != 3)
    {
        throw py::value_error("The image must have 3 dimensions.");
    }
    int height = static_cast<int>(bgrImg.shape(0));
    int width = bgrImg.shape(1);
    size_t channelCount = bgrImg.shape(2);
    if (channelCount != 3)
    {
        throw py::value_error("The channel count must be 3.");
    }

    self->sendFrame(
        cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(bgrImg.data()), bgrImg.strides(0)),
        timestampUs);
}

void opentera::initVideoSourcePython(pybind11::module& m)
{
    py::class_<VideoSource, shared_ptr<VideoSource>>(
        m,
        "VideoSource",
        "Represents a video source that can be added to a WebRTC call.\n"
        "\n"
        "Pass an instance of this to the StreamClient and call sendFrame for "
        "each of your frame.")
        .def(
            py::init<VideoSourceConfiguration>(),
            "Creates a VideoSource\n"
            "\n"
            ":param configuration: The configuration applied to the video "
            "stream by the image transport layer",
            py::arg("configuration"))
        .def(
            "send_frame",
            &sendFrame,
            py::call_guard<py::gil_scoped_release>(),
            "Sends a frame to the WebRTC transport layer\n"
            "\n"
            "The frame may or may not be sent depending of the transport layer "
            "state\n"
            "Frame will be resized to match the transport layer request\n"
            "\n"
            ":param bgr_img: BGR8 encoded frame data\n"
            ":param timestamp_us: Frame timestamp in microseconds",
            py::arg("bgr_img"),
            py::arg("timestamp_us"));
}
