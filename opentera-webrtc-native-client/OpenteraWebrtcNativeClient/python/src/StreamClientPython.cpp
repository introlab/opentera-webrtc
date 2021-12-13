#include <OpenteraWebrtcNativeClientPython/StreamClientPython.h>

#include <OpenteraWebrtcNativeClient/StreamClient.h>

#include <pybind11/functional.h>
#include <pybind11/numpy.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void setOnVideoFrameReceived(StreamClient& self,
        const function<void(const Client&, const py::array_t<uint8_t>&, uint64_t)>& pythonCallback)
{
    auto callback = [=](const Client& client, const cv::Mat& bgrImg, uint64_t timestampUs)
    {
        size_t height = bgrImg.rows;
        size_t width = bgrImg.cols;
        size_t channelCount = bgrImg.channels();

        py::buffer_info bufferInfo(bgrImg.data,
                sizeof(uint8_t),
                py::format_descriptor<uint8_t>::format(),
                3, // Number of dimensions
                { height, width, channelCount }, // Buffer dimensions
                // Strides (in bytes) for each index
                { bgrImg.step[0], bgrImg.step[1], sizeof(uint8_t) },
                true); // Readonly

        py::gil_scoped_acquire acquire;
        py::array_t<uint8_t> numpyBgrImg(bufferInfo);
        pythonCallback(client, numpyBgrImg, timestampUs);
    };

    self.setOnVideoFrameReceived(callback);
}

py::buffer_info getAudioBufferInfo(const void* audioData,
        int bitsPerSample, size_t numberOfChannels, size_t numberOfFrames)
{
    py::buffer_info bufferInfo;
    switch (bitsPerSample)
    {
    case 8:
    bufferInfo = py::buffer_info(const_cast<void*>(audioData),
            sizeof(int8_t),
            py::format_descriptor<int8_t>::format(),
            1, // Number of dimensions
            { numberOfFrames * numberOfChannels }, // Buffer dimensions
            { sizeof(int8_t) }); // Strides (in bytes) for each index
    break;

    case 16:
    bufferInfo = py::buffer_info(const_cast<void*>(audioData),
            sizeof(int16_t),
            py::format_descriptor<int16_t>::format(),
            1, // Number of dimensions
            { numberOfFrames * numberOfChannels }, // Buffer dimensions
            { sizeof(int16_t) }); // Strides (in bytes) for each index
    break;

    case 32:
    bufferInfo = py::buffer_info(const_cast<void*>(audioData),
            sizeof(int32_t),
            py::format_descriptor<int32_t>::format(),
            1, // Number of dimensions
            { numberOfFrames * numberOfChannels }, // Buffer dimensions
            { sizeof(int32_t) }); // Strides (in bytes) for each index
    break;

    default:
    break;
    }

    return bufferInfo;
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
        py::buffer_info bufferInfo = getAudioBufferInfo(audioData, bitsPerSample, numberOfChannels, numberOfFrames);
        py::gil_scoped_acquire acquire;
        pythonCallback(client, py::array(bufferInfo), sampleRate, numberOfChannels, numberOfFrames);
    };

    self.setOnAudioFrameReceived(callback);
}

void setOnMixedAudioFrameReceived(StreamClient& self,
        const function<void(const py::array&, int, size_t, size_t)>& pythonCallback)
{
    auto callback = [=](
            const void* audioData,
            int bitsPerSample,
            int sampleRate,
            size_t numberOfChannels,
            size_t numberOfFrames)
    {
        py::buffer_info bufferInfo = getAudioBufferInfo(audioData, bitsPerSample, numberOfChannels, numberOfFrames);
        py::gil_scoped_acquire acquire;
        pythonCallback(py::array(bufferInfo), sampleRate, numberOfChannels, numberOfFrames);
    };

    self.setOnMixedAudioFrameReceived(callback);
}

void opentera::initStreamClientPython(pybind11::module& m)
{
    py::class_<StreamClient, SignalingClient>(m, "StreamClient",
            "A signaling client to join a WebRTC room and stream a video source.")
            .def(py::init<SignalingServerConfiguration, WebrtcConfiguration>(),
                    "Creates a stream client\n"
                    "\n"
                    ":param signaling_server_configuration: The configuration to connect to the signaling server\n"
                    ":param webrtc_configuration: The WebRTC configuration\n"
                    " */",
                    py::arg("signaling_server_configuration"), py::arg("webrtc_configuration"))
            .def(py::init<SignalingServerConfiguration, WebrtcConfiguration, shared_ptr<VideoSource>>(),
                    "Creates a stream client\n"
                    "\n"
                    ":param signaling_server_configuration: The configuration to connect to the signaling server\n"
                    ":param webrtc_configuration: The WebRTC configuration\n"
                    ":param video_source: The video source that this client will add to the call",
                    py::arg("signaling_server_configuration"), py::arg("webrtc_configuration"), py::arg("video_source"))
            .def(py::init<SignalingServerConfiguration, WebrtcConfiguration, shared_ptr<AudioSource>>(),
                    "Creates a stream client\n"
                    "\n"
                    ":param signaling_server_configuration: The configuration to connect to the signaling server\n"
                    ":param webrtc_configuration: The WebRTC configuration\n"
                    ":param audio_source: The audio source that this client will add to the call",
                    py::arg("signaling_server_configuration"), py::arg("webrtc_configuration"), py::arg("audio_source"))
            .def(py::init<SignalingServerConfiguration, WebrtcConfiguration, shared_ptr<VideoSource>, shared_ptr<AudioSource>>(),
                    "Creates a stream client\n"
                    "\n"
                    ":param signaling_server_configuration: The configuration to connect to the signaling server\n"
                    ":param webrtc_configuration: The WebRTC configuration\n"
                    ":param video_source: The video source that this client will add to the call\n"
                    ":param audio_source: The audio source that this client will add to the call",
                    py::arg("signaling_server_configuration"), py::arg("webrtc_configuration"), py::arg("video_source"), py::arg("audio_source"))

            .def_property("is_local_audio_muted", &StreamClient::isLocalAudioMuted, &StreamClient::setLocalAudioMuted,
                    "Indicates if the local audio is muted.")
            .def("mute_local_audio", &StreamClient::muteLocalAudio, "Mutes the local audio.")
            .def("unmute_local_audio", &StreamClient::unmuteLocalAudio, "Unmutes the local audio.")

            .def_property("is_local_video_muted", &StreamClient::isLocalVideoMuted, &StreamClient::setLocalVideoMuted,
                          "Indicates if the local video is muted.")
            .def("mute_local_video", &StreamClient::muteLocalVideo, "Mutes the local video.")
            .def("unmute_local_video", &StreamClient::unmuteLocalVideo, "Unmutes the local video.")

            .def_property("on_add_remote_stream", nullptr, &StreamClient::setOnAddRemoteStream,
                    "Sets the callback that is called when a stream is added.\n"
                    "\n"
                    "The callback is called from the internal client thread.\n"
                    "\n"
                    "Callback parameters:\n"
                    " - client: The client of the stream\n"
                    "\n"
                    ":param callback: The callback")
            .def_property("on_remove_remote_stream", nullptr, &StreamClient::setOnRemoveRemoteStream,
                    "Sets the callback that is called when a stream is removed.\n"
                    "\n"
                    "The callback is called from the internal client thread.\n"
                    "\n"
                    "Callback parameters:\n"
                    " - client: The client of the stream\n"
                    "\n"
                    ":param callback: The callback")
            .def_property("on_video_frame_received", nullptr, &setOnVideoFrameReceived,
                    "Sets the callback that is called when a video stream frame is received.\n"
                    "\n"
                    "The callback is called from a WebRTC processing thread.\n"
                    "\n"
                    "Callback parameters:\n"
                    " - client: The client of the stream frame\n"
                    " - bgr_img: The BGR frame image (numpy.array[uint8])\n"
                    " - timestamp_us The timestamp in microseconds\n"
                    "\n"
                    ":param callback: The callback")
            .def_property("on_audio_frame_received", nullptr, &setOnAudioFrameReceived,
                    "Sets the callback that is called when an audio stream frame is received.\n"
                    "\n"
                    "The callback is called from a WebRTC processing thread.\n"
                    "\n"
                    "Callback parameters:\n"
                    " - client: The client of the stream frame\n"
                    " - audio_data: The audio data (numpy.array[int8], numpy.array[int16] or numpy.array[int32])\n"
                    " - sample_rate: The audio stream sample rate\n"
                    " - number_of_channels: The audio stream channel count\n"
                    " - number_of_frames: The number of frames\n"
                    "\n"
                    ":param callback: The callback")
            .def_property("on_mixed_audio_frame_received", nullptr, &setOnMixedAudioFrameReceived,
                    "Sets the callback that is called when a mixed audio stream frame is received.\n"
                    "\n"
                    "The callback is called from a WebRTC processing thread.\n"
                    "\n"
                    "Callback parameters:\n"
                    " - audio_data: The audio data (numpy.array[int8], numpy.array[int16] or numpy.array[int32])\n"
                    " - sample_rate: The audio stream sample rate\n"
                    " - number_of_channels: The audio stream channel count\n"
                    " - number_of_frames: The number of frames\n"
                    "\n"
                    ":param callback: The callback");
}
