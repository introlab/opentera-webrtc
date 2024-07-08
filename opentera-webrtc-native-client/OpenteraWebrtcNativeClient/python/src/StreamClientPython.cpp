#include <OpenteraWebrtcNativeClientPython/PyBindUtils.h>
#include <OpenteraWebrtcNativeClientPython/StreamClientPython.h>

#include <OpenteraWebrtcNativeClient/StreamClient.h>

#include <pybind11/functional.h>
#include <pybind11/numpy.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void setOnVideoFrameReceived(
    StreamClient& self,
    const function<void(const Client&, const py::array_t<uint8_t>&, uint64_t)>& pythonCallback)
{
    auto callback = [=](const Client& client, const cv::Mat& bgrImg, uint64_t timestampUs)
    {
        size_t height = bgrImg.rows;
        size_t width = bgrImg.cols;
        size_t channelCount = bgrImg.channels();

        py::buffer_info bufferInfo(
            bgrImg.data,
            sizeof(uint8_t),
            py::format_descriptor<uint8_t>::format(),
            3,  // Number of dimensions
            {height, width, channelCount},  // Buffer dimensions
            // Strides (in bytes) for each index
            {bgrImg.step[0], bgrImg.step[1], sizeof(uint8_t)},
            true);  // Readonly

        py::gil_scoped_acquire acquire;
        py::array_t<uint8_t> numpyBgrImg(bufferInfo);
        pythonCallback(client, numpyBgrImg, timestampUs);
    };

    self.setOnVideoFrameReceived(callback);
}

py::buffer_info
    getAudioBufferInfo(const void* audioData, int bitsPerSample, size_t numberOfChannels, size_t numberOfFrames)
{
    py::buffer_info bufferInfo;
    switch (bitsPerSample)
    {
        case 8:
            bufferInfo = py::buffer_info(
                const_cast<void*>(audioData),
                sizeof(int8_t),
                py::format_descriptor<int8_t>::format(),
                1,  // Number of dimensions
                {numberOfFrames * numberOfChannels},  // Buffer dimensions
                {sizeof(int8_t)});  // Strides (in bytes) for each index
            break;

        case 16:
            bufferInfo = py::buffer_info(
                const_cast<void*>(audioData),
                sizeof(int16_t),
                py::format_descriptor<int16_t>::format(),
                1,  // Number of dimensions
                {numberOfFrames * numberOfChannels},  // Buffer dimensions
                {sizeof(int16_t)});  // Strides (in bytes) for each index
            break;

        case 32:
            bufferInfo = py::buffer_info(
                const_cast<void*>(audioData),
                sizeof(int32_t),
                py::format_descriptor<int32_t>::format(),
                1,  // Number of dimensions
                {numberOfFrames * numberOfChannels},  // Buffer dimensions
                {sizeof(int32_t)});  // Strides (in bytes) for each index
            break;

        default:
            break;
    }

    return bufferInfo;
}

void setOnEncodedVideoFrameReceived(
    StreamClient& self,
    const function<void(
        const Client& client,
        const py::bytes& data,
        VideoCodecType codecType,
        bool isKeyFrame,
        uint32_t width,
        uint32_t height,
        uint64_t timestampUs)>& pythonCallback)
{
    auto callback = [=](const Client& client,
                        const uint8_t* data,
                        size_t dataSize,
                        VideoCodecType codecType,
                        bool isKeyFrame,
                        uint32_t width,
                        uint32_t height,
                        uint64_t timestampUs)
    {
        py::gil_scoped_acquire acquire;
        py::bytes dataBytes(reinterpret_cast<const char*>(data), dataSize);
        pythonCallback(client, dataBytes, codecType, isKeyFrame, width, height, timestampUs);
    };

    self.setOnEncodedVideoFrameReceived(callback);
}

void setOnAudioFrameReceived(
    StreamClient& self,
    const function<void(const Client&, const py::array&, int, size_t, size_t)>& pythonCallback)
{
    auto callback = [=](const Client& client,
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

void setOnMixedAudioFrameReceived(
    StreamClient& self,
    const function<void(const py::array&, int, size_t, size_t)>& pythonCallback)
{
    auto callback =
        [=](const void* audioData, int bitsPerSample, int sampleRate, size_t numberOfChannels, size_t numberOfFrames)
    {
        py::buffer_info bufferInfo = getAudioBufferInfo(audioData, bitsPerSample, numberOfChannels, numberOfFrames);
        py::gil_scoped_acquire acquire;
        pythonCallback(py::array(bufferInfo), sampleRate, numberOfChannels, numberOfFrames);
    };

    self.setOnMixedAudioFrameReceived(callback);
}

void opentera::initStreamClientPython(pybind11::module& m)
{
    py::enum_<VideoCodecType>(m, "VideoCodecType")
        .value("GENERIC", VideoCodecType::Generic)
        .value("VP8", VideoCodecType::VP8)
        .value("VP9", VideoCodecType::VP9)
        .value("AV1", VideoCodecType::AV1)
        .value("H264", VideoCodecType::H264)
        .value("MULTIPLEX", VideoCodecType::Multiplex);

    py::class_<StreamClient, WebrtcClient>(
        m,
        "StreamClient",
        "A signaling client to join a WebRTC room and stream a video source.")
        .def(
            py::init<SignalingServerConfiguration, WebrtcConfiguration, VideoStreamConfiguration>(),
            "Creates a stream client\n"
            "\n"
            ":param signaling_server_configuration: The configuration to "
            "connect to the signaling server\n"
            ":param webrtc_configuration: The WebRTC configuration\n"
            ":param video_stream_configuration: The video stream configuration\n",
            py::arg("signaling_server_configuration"),
            py::arg("webrtc_configuration"),
            py::arg("video_stream_configuration"))
        .def(
            py::init<
                SignalingServerConfiguration,
                WebrtcConfiguration,
                VideoStreamConfiguration,
                shared_ptr<VideoSource>>(),
            "Creates a stream client\n"
            "\n"
            ":param signaling_server_configuration: The configuration to "
            "connect to the signaling server\n"
            ":param webrtc_configuration: The WebRTC configuration\n"
            ":param video_stream_configuration: The video stream configuration\n"
            ":param video_source: The video source that this client will add to "
            "the call",
            py::arg("signaling_server_configuration"),
            py::arg("webrtc_configuration"),
            py::arg("video_stream_configuration"),
            py::arg("video_source"))
        .def(
            py::init<
                SignalingServerConfiguration,
                WebrtcConfiguration,
                VideoStreamConfiguration,
                shared_ptr<AudioSource>>(),
            "Creates a stream client\n"
            "\n"
            ":param signaling_server_configuration: The configuration to "
            "connect to the signaling server\n"
            ":param webrtc_configuration: The WebRTC configuration\n"
            ":param video_stream_configuration: The video stream configuration\n"
            ":param audio_source: The audio source that this client will add to "
            "the call",
            py::arg("signaling_server_configuration"),
            py::arg("webrtc_configuration"),
            py::arg("video_stream_configuration"),
            py::arg("audio_source"))
        .def(
            py::init<
                SignalingServerConfiguration,
                WebrtcConfiguration,
                VideoStreamConfiguration,
                shared_ptr<VideoSource>,
                shared_ptr<AudioSource>>(),
            "Creates a stream client\n"
            "\n"
            ":param signaling_server_configuration: The configuration to "
            "connect to the signaling server\n"
            ":param webrtc_configuration: The WebRTC configuration\n"
            ":param video_stream_configuration: The video stream configuration\n"
            ":param video_source: The video source that this client will add to "
            "the call\n"
            ":param audio_source: The audio source that this client will add to "
            "the call",
            py::arg("signaling_server_configuration"),
            py::arg("webrtc_configuration"),
            py::arg("video_stream_configuration"),
            py::arg("video_source"),
            py::arg("audio_source"))

        .def_property(
            "is_local_audio_muted",
            GilScopedRelease<StreamClient>::guard(&StreamClient::isLocalAudioMuted),
            GilScopedRelease<StreamClient>::guard(&StreamClient::setLocalAudioMuted),
            "Indicates if the local audio is muted.")
        .def(
            "mute_local_audio",
            &StreamClient::muteLocalAudio,
            py::call_guard<py::gil_scoped_release>(),
            "Mutes the local audio.")
        .def(
            "unmute_local_audio",
            &StreamClient::unmuteLocalAudio,
            py::call_guard<py::gil_scoped_release>(),
            "Unmutes the local audio.")

        .def_property(
            "is_remote_audio_muted",
            GilScopedRelease<StreamClient>::guard(&StreamClient::isRemoteAudioMuted),
            GilScopedRelease<StreamClient>::guard(&StreamClient::setRemoteAudioMuted),
            "Indicates if the remote audio is muted.")
        .def(
            "mute_remote_audio",
            &StreamClient::muteRemoteAudio,
            py::call_guard<py::gil_scoped_release>(),
            "Mutes the remote audio.")
        .def(
            "unmute_remote_audio",
            &StreamClient::unmuteRemoteAudio,
            py::call_guard<py::gil_scoped_release>(),
            "Unmutes the remote audio.")

        .def_property(
            "is_local_video_muted",
            GilScopedRelease<StreamClient>::guard(&StreamClient::isLocalVideoMuted),
            GilScopedRelease<StreamClient>::guard(&StreamClient::setLocalVideoMuted),
            "Indicates if the local video is muted.")
        .def(
            "mute_local_video",
            &StreamClient::muteLocalVideo,
            py::call_guard<py::gil_scoped_release>(),
            "Mutes the local video.")
        .def(
            "unmute_local_video",
            &StreamClient::unmuteLocalVideo,
            py::call_guard<py::gil_scoped_release>(),
            "Unmutes the local video.")

        .def_property(
            "on_add_remote_stream",
            nullptr,
            GilScopedRelease<StreamClient>::guard(&StreamClient::setOnAddRemoteStream),
            "Sets the callback that is called when a stream is added.\n"
            "\n"
            "The callback is called from the internal client thread. "
            "The callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client of the stream\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_remove_remote_stream",
            nullptr,
            GilScopedRelease<StreamClient>::guard(&StreamClient::setOnRemoveRemoteStream),
            "Sets the callback that is called when a stream is removed.\n"
            "\n"
            "The callback is called from the internal client thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client of the stream\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_video_frame_received",
            nullptr,
            GilScopedRelease<StreamClient>::guard(&setOnVideoFrameReceived),
            "Sets the callback that is called when a video stream frame is "
            "received.\n"
            "\n"
            "The callback is called from a WebRTC processing thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client of the stream frame\n"
            " - bgr_img: The BGR frame image (numpy.array[uint8])\n"
            " - timestamp_us The timestamp in microseconds\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_encoded_video_frame_received",
            nullptr,
            GilScopedRelease<StreamClient>::guard(&setOnEncodedVideoFrameReceived),
            "Sets the callback that is called when an encoded video "
            "stream frame is received.\n"
            "\n"
            "The callback is called from a WebRTC processing thread. "
            "The callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client of the stream frame\n"
            " - data: The binary data (bytes)\n"
            " - codec_type: The codec type\n"
            " - is_key_frame: Indicates if it is a key frame\n"
            " - width: The frame width if it is a key frame\n"
            " - height: The frame height if it is a key frame\n"
            " - timestamp_us The timestamp in microseconds\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_audio_frame_received",
            nullptr,
            GilScopedRelease<StreamClient>::guard(&setOnAudioFrameReceived),
            "Sets the callback that is called when an audio stream frame is "
            "received.\n"
            "\n"
            "The callback is called from a WebRTC processing thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client of the stream frame\n"
            " - audio_data: The audio data (numpy.array[int8], "
            "numpy.array[int16] or numpy.array[int32])\n"
            " - sample_rate: The audio stream sample rate\n"
            " - number_of_channels: The audio stream channel count\n"
            " - number_of_frames: The number of frames\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_mixed_audio_frame_received",
            nullptr,
            GilScopedRelease<StreamClient>::guard(&setOnMixedAudioFrameReceived),
            "Sets the callback that is called when a mixed audio stream frame is "
            "received.\n"
            "\n"
            "The callback is called from a WebRTC processing thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - audio_data: The audio data (numpy.array[int8], "
            "numpy.array[int16] or numpy.array[int32])\n"
            " - sample_rate: The audio stream sample rate\n"
            " - number_of_channels: The audio stream channel count\n"
            " - number_of_frames: The number of frames\n"
            "\n"
            ":param callback: The callback");
}
