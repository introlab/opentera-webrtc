#include <OpenteraWebrtcNativeClientPython/VideoSourcePython.h>

#include <OpenteraWebrtcNativeClient/VideoSource.h>

#include <pybind11/numpy.h>

using namespace introlab;
using namespace std;
namespace py = pybind11;

void sendFrame(const std::shared_ptr<VideoSource>& self, py::array_t<uint8_t>& bgrImg, int64_t timestampUs)
{
    if (bgrImg.ndim() != 3)
    {
        throw py::value_error("The image must have 3 dimensions.");
    }
    int height = static_cast<int>(bgrImg.shape(0));
    int width = bgrImg.shape(1);
    size_t channel_count = bgrImg.shape(2);
    if (channel_count != 3)
    {
        throw py::value_error("The channel count must be 3.");
    }

    self->sendFrame(cv::Mat(height, width, CV_8UC3, bgrImg.mutable_data(), bgrImg.strides(0)), timestampUs);
}

void introlab::initVideoSourcePython(pybind11::module& m)
{
    py::class_<VideoSource, shared_ptr<VideoSource>>(m, "VideoSource")
        .def(py::init<bool, bool>(), py::arg("needs_denoising"), py::arg("is_screencast"))
        .def("send_frame", &sendFrame, py::arg("bgr_img"), py::arg("timestamp_us"));
}
