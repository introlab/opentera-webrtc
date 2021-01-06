#include <OpenteraWebrtcNativeClientPython/DataChannelClientPython.h>

#include <OpenteraWebrtcNativeClient/DataChannelClient.h>

#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initDataChannelClientPython(pybind11::module& m)
{
    py::class_<DataChannelClient, SignalingClient>(m, "DataChannelClient")
            .def(py::init<SignalingServerConfiguration, WebrtcConfiguration, DataChannelConfiguration>(),
                    py::arg("signaling_server_configuration"), py::arg("webrtc_configuration"), py::arg("data_channel_configuration"))

            .def("send_to", [](DataChannelClient& self, py::bytes bytes, const vector<string>& ids)
            {
                auto data = py::bytes(bytes).cast<string>();
                self.sendTo(reinterpret_cast<const uint8_t*>(data.data()), data.size(), ids);
            }, py::arg("bytes"), py::arg("ids"))
            .def("send_to",
                    py::overload_cast<const string&, const vector<string>&>(&DataChannelClient::sendTo),
                    py::arg("message"), py::arg("ids"))
            .def("send_to_all", [](DataChannelClient& self, py::bytes bytes)
            {
                auto data = py::bytes(bytes).cast<string>();
                self.sendToAll(reinterpret_cast<const uint8_t*>(data.data()), data.size());
            }, py::arg("bytes"))
            .def("send_to_all", py::overload_cast<const string&>(&DataChannelClient::sendToAll),
                    py::arg("message"))

            .def_property("on_data_channel_open", nullptr, &DataChannelClient::setOnDataChannelOpen)
            .def_property("on_data_channel_closed", nullptr, &DataChannelClient::setOnDataChannelClosed)
            .def_property("on_data_channel_error", nullptr, &DataChannelClient::setOnDataChannelError)
            .def_property("on_data_channel_message_binary", nullptr, &DataChannelClient::setOnDataChannelMessageBinary)
            .def_property("on_data_channel_message_string", nullptr, &DataChannelClient::setOnDataChannelMessageString);
}
