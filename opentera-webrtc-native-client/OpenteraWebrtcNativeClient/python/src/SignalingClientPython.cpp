#include <OpenteraWebrtcNativeClientPython/SignalingClientPython.h>

#include <OpenteraWebrtcNativeClient/SignalingClient.h>

#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initSignalingClientPython(pybind11::module& m)
{
    py::class_<SignalingClient>(m, "SignalingClient")
            .def("connect", &SignalingClient::connect)
            .def("close", &SignalingClient::close)
            .def("close_sync", &SignalingClient::closeSync)

            .def("call_all", &SignalingClient::callAll)
            .def("call_ids", &SignalingClient::callIds, py::arg("ids"))

            .def("hang_up_all", &SignalingClient::hangUpAll)
            .def("close_all_room_peer_connections", &SignalingClient::closeAllRoomPeerConnections)

            .def_property_readonly("is_connected", &SignalingClient::isConnected)
            .def_property_readonly("is_rtc_connected", &SignalingClient::isRtcConnected)
            .def_property_readonly("id", &SignalingClient::id)

            .def_property_readonly("connected_room_client_ids", &SignalingClient::getConnectedRoomClientIds)

            .def("get_room_client", &SignalingClient::getRoomClient, py::arg("id"))
            .def_property_readonly("room_clients", &SignalingClient::getRoomClients)

            .def_property("on_signaling_connection_open", nullptr, &SignalingClient::setOnSignalingConnectionOpen)
            .def_property("on_signaling_connection_closed", nullptr, &SignalingClient::setOnSignalingConnectionClosed)
            .def_property("on_signaling_connection_error", nullptr, &SignalingClient::setOnSignalingConnectionError)

            .def_property("on_room_clients_changed", nullptr, &SignalingClient::setOnRoomClientsChanged)

            .def_property("call_acceptor", nullptr, &SignalingClient::setCallAcceptor)
            .def_property("on_call_rejected", nullptr, &SignalingClient::setOnCallRejected)

            .def_property("on_client_connected", nullptr, &SignalingClient::setOnClientConnected)
            .def_property("on_client_disconnected", nullptr, &SignalingClient::setOnClientDisconnected)

            .def_property("on_error", nullptr, &SignalingClient::setOnError);
}
