#include <OpenteraWebrtcNativeClientPython/SignallingClientPython.h>

#include <OpenteraWebrtcNativeClient/SignallingClient.h>

#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace introlab;
using namespace std;
namespace py = pybind11;

void introlab::initSignallingClientPython(pybind11::module& m)
{
    py::class_<SignallingClient>(m, "SignallingClient")
            .def("connect", &SignallingClient::connect)
            .def("close", &SignallingClient::close)
            .def("close_sync", &SignallingClient::closeSync)

            .def("call_all", &SignallingClient::callAll)
            .def("call_ids", &SignallingClient::callIds, py::arg("ids"))

            .def("hang_up_all", &SignallingClient::hangUpAll)
            .def("close_all_room_peer_connections", &SignallingClient::closeAllRoomPeerConnections)

            .def_property_readonly("is_connected", &SignallingClient::isConnected)
            .def_property_readonly("is_rtc_connected", &SignallingClient::isRtcConnected)
            .def_property_readonly("id", &SignallingClient::id)

            .def_property_readonly("connected_room_client_ids", &SignallingClient::getConnectedRoomClientIds)

            .def("get_room_client", &SignallingClient::getRoomClient, py::arg("id"))
            .def_property_readonly("room_clients", &SignallingClient::getRoomClients)

            .def_property("on_signalling_connection_open", nullptr, &SignallingClient::setOnSignallingConnectionOpen)
            .def_property("on_signalling_connection_closed", nullptr, &SignallingClient::setOnSignallingConnectionClosed)
            .def_property("on_signalling_connection_error", nullptr, &SignallingClient::setOnSignallingConnectionError)

            .def_property("on_room_clients_changed", nullptr, &SignallingClient::setOnRoomClientsChanged)

            .def_property("call_acceptor", nullptr, &SignallingClient::setCallAcceptor)
            .def_property("on_call_rejected", nullptr, &SignallingClient::setOnCallRejected)

            .def_property("on_client_connected", nullptr, &SignallingClient::setOnClientConnected)
            .def_property("on_client_disconnected", nullptr, &SignallingClient::setOnClientDisconnected)

            .def_property("on_error", nullptr, &SignallingClient::setOnError);
}
