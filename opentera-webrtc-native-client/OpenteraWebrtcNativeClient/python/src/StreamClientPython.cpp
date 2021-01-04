#include <OpenteraWebrtcNativeClientPython/StreamClientPython.h>

#include <OpenteraWebrtcNativeClient/StreamClient.h>

#include <pybind11/numpy.h>

using namespace introlab;
using namespace std;
namespace py = pybind11;

void setOnVideoFrameReceived(StreamClient& self,
        const function<void(const Client&, const py::array_t<uint8_t>&, uint64_t)>& pythonCallback)
{
    auto callback = [=](const Client& client, const cv::Mat& bgrImg, uint64_t timestampUs)
    {
        size_t height = bgrImg.rows;
        size_t width = bgrImg.cols;
        size_t channelCount = 3;

        py::buffer_info bufferInfo(bgrImg.data,
                sizeof(uint8_t),
                py::format_descriptor<uint8_t>::format(),
                3, // Number of dimensions
                { height, width, channelCount }, // Buffer dimensions
                // Strides (in bytes) for each index
                { width * channelCount * sizeof(uint8_t), channelCount * sizeof(uint8_t), sizeof(uint8_t) });
        py::array_t<uint8_t> numpyBgrImg(bufferInfo);
        pythonCallback(client, numpyBgrImg, timestampUs);
    };

    self.setOnVideoFrameReceived(callback);
}

void setOnAudioFrameReceived(StreamClient& self,
        const function<void(const Client&, const py::array&, int, size_t, size_t)>& pythonCallback)
{
    auto callback = [=](
            const Client& client,
            const void* audioData,
            int bitsPerSample,
            int sampleRate,
            size_t numberOfChannels,
            size_t numberOfFrames)
    {
        py::buffer_info bufferInfo;
        switch (bitsPerSample)
        {
            case 8:
                bufferInfo = py::buffer_info(const_cast<void*>(audioData),
                        sizeof(uint8_t),
                        py::format_descriptor<uint8_t>::format(),
                        1, // Number of dimensions
                        { numberOfFrames * numberOfChannels }, // Buffer dimensions
                        { sizeof(uint8_t) }); // Strides (in bytes) for each index
                break;

            case 16:
                bufferInfo = py::buffer_info(const_cast<void*>(audioData),
                        sizeof(uint16_t),
                        py::format_descriptor<uint16_t>::format(),
                        1, // Number of dimensions
                        { numberOfFrames * numberOfChannels }, // Buffer dimensions
                        { sizeof(uint16_t) }); // Strides (in bytes) for each index
                break;

            case 32:
                bufferInfo = py::buffer_info(const_cast<void*>(audioData),
                        sizeof(uint32_t),
                        py::format_descriptor<uint32_t>::format(),
                        1, // Number of dimensions
                        { numberOfFrames * numberOfChannels }, // Buffer dimensions
                        { sizeof(uint32_t) }); // Strides (in bytes) for each index
                break;

            default:
                break;
        }

        pythonCallback(client, py::array(bufferInfo), sampleRate, numberOfChannels, numberOfFrames);
    };

    self.setOnAudioFrameReceived(callback);
}

void introlab::initStreamClientPython(pybind11::module& m)
{
    py::class_<StreamClient, SignallingClient>(m, "StreamClient")
        .def(py::init<SignallingServerConfiguration, WebrtcConfiguration>(),
             py::arg("signalling_server_configuration"), py::arg("webrtc_configuration"))
        .def(py::init<SignallingServerConfiguration, WebrtcConfiguration, shared_ptr<VideoSource>>(),
             py::arg("signalling_server_configuration"), py::arg("webrtc_configuration"), py::arg("video_source"))
        .def(py::init<SignallingServerConfiguration, WebrtcConfiguration, shared_ptr<AudioSource>>(),
             py::arg("signalling_server_configuration"), py::arg("webrtc_configuration"), py::arg("audio_source"))
        .def(py::init<SignallingServerConfiguration, WebrtcConfiguration, shared_ptr<VideoSource>, shared_ptr<AudioSource>>(),
             py::arg("signalling_server_configuration"), py::arg("webrtc_configuration"), py::arg("video_source"), py::arg("audio_source"))

        .def_property("on_add_remote_stream", nullptr, &StreamClient::setOnAddRemoteStream)
        .def_property("on_remove_remote_stream", nullptr, &StreamClient::setOnRemoveRemoteStream)
        .def_property("on_video_frame_received", nullptr, &setOnVideoFrameReceived)
        .def_property("on_audio_frame_received", nullptr, &setOnAudioFrameReceived);
}
