#include <OpenteraWebrtcNativeClientPython/DataChannelClientPython.h>
#include <OpenteraWebrtcNativeClientPython/PyBindUtils.h>

#include <OpenteraWebrtcNativeClient/DataChannelClient.h>

#include <pybind11/functional.h>
#include <pybind11/stl.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void setOnDataChannelMessageBinary(
    DataChannelClient& self,
    const function<void(const Client&, const py::bytes&)>& pythonCallback)
{
    auto callback = [=](const Client& client, const uint8_t* data, size_t dataSize)
    {
        py::gil_scoped_acquire acquire;
        pythonCallback(client, py::bytes(reinterpret_cast<const char*>(data), dataSize));
    };

    self.setOnDataChannelMessageBinary(callback);
}

void opentera::initDataChannelClientPython(pybind11::module& m)
{
    py::class_<DataChannelClient, WebrtcClient>(
        m,
        "DataChannelClient",
        "Represents a client for data channel communication.")
        .def(
            py::init<SignalingServerConfiguration, WebrtcConfiguration, DataChannelConfiguration>(),
            "Creates a data channel client with the specified configurations.\n"
            "\n"
            ":param signaling_server_configuration: The signaling server "
            "configuration\n"
            ":param webrtc_configuration: The WebRTC configuration\n"
            ":param data_channel_configuration: The data channel configuration",
            py::arg("signaling_server_configuration"),
            py::arg("webrtc_configuration"),
            py::arg("data_channel_configuration"))

        .def(
            "send_to",
            [](DataChannelClient& self, const py::bytes& bytes, const vector<string>& ids)
            {
                auto data = bytes.cast<string>();
                self.sendTo(reinterpret_cast<const uint8_t*>(data.data()), data.size(), ids);
            },
            py::call_guard<py::gil_scoped_release>(),
            "Sends binary data to the specified clients.\n"
            "\n"
            ":param bytes: The binary data\n"
            ":param ids: The client ids",
            py::arg("bytes"),
            py::arg("ids"))
        .def(
            "send_to",
            py::overload_cast<const string&, const vector<string>&>(&DataChannelClient::sendTo),
            py::call_guard<py::gil_scoped_release>(),
            "Sends a string message to the specified clients.\n"
            "\n"
            ":param message: The string message\n"
            ":param ids: The client ids",
            py::arg("message"),
            py::arg("ids"))
        .def(
            "send_to_all",
            [](DataChannelClient& self, const py::bytes& bytes)
            {
                auto data = bytes.cast<string>();
                self.sendToAll(reinterpret_cast<const uint8_t*>(data.data()), data.size());
            },
            py::call_guard<py::gil_scoped_release>(),
            "Sends binary data to all clients.\n"
            "\n"
            ":param bytes: The binary data (bytes)\n",
            py::arg("bytes"))
        .def(
            "send_to_all",
            py::overload_cast<const string&>(&DataChannelClient::sendToAll),
            py::call_guard<py::gil_scoped_release>(),
            "Sends a string message to all clients.\n"
            "\n"
            ":param message: The string message",
            py::arg("message"))

        .def_property(
            "on_data_channel_opened",
            nullptr,
            GilScopedRelease<DataChannelClient>::guard(&DataChannelClient::setOnDataChannelOpened),
            "Sets the callback that is called when a data channel opens.\n"
            "\n"
            "The callback is called from the internal client thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client of the data channel that opens\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_data_channel_closed",
            nullptr,
            GilScopedRelease<DataChannelClient>::guard(&DataChannelClient::setOnDataChannelClosed),
            "Sets the callback that is called when a data channel closes.\n"
            "\n"
            "The callback is called from the internal client thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client of the data channel that closes\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_data_channel_error",
            nullptr,
            GilScopedRelease<DataChannelClient>::guard(&DataChannelClient::setOnDataChannelError),
            "Sets the callback that is called when a data channel error occurs.\n"
            "\n"
            "The callback is called from the internal client thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client of the data channel error\n"
            " - error: The error message\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_data_channel_message_binary",
            nullptr,
            GilScopedRelease<DataChannelClient>::guard(&setOnDataChannelMessageBinary),
            "Sets the callback that is called when binary data are received.\n"
            "\n"
            "The callback is called from the internal client thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client the binary data are from\n"
            " - bytes: The binary data\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_data_channel_message_string",
            nullptr,
            GilScopedRelease<DataChannelClient>::guard(&DataChannelClient::setOnDataChannelMessageString),
            "Sets the callback that is called when a string message is "
            "received.\n"
            "\n"
            "The callback is called from the internal client thread. "
            "The callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client the binary data is from\n"
            " - message: The string message\n"
            "\n"
            ":param callback: The callback");
}
