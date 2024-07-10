#include <OpenteraWebrtcNativeClientPython/PyBindUtils.h>
#include <OpenteraWebrtcNativeClientPython/WebrtcClientPython.h>

#include <OpenteraWebrtcNativeClient/WebrtcClient.h>

#include <pybind11/functional.h>
#include <pybind11/stl.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initWebrtcClientPython(pybind11::module& m)
{
    py::class_<WebrtcClient>(m, "WebrtcClient")
        .def("connect", &WebrtcClient::connect, "Connects the client the signaling server.")
        .def("close", &WebrtcClient::close, "Closes all client connections (asynchronous).")
        .def(
            "close_sync",
            &WebrtcClient::closeSync,
            py::call_guard<py::gil_scoped_release>(),
            "Closes all client connections (synchronous).")

        .def("call_all", &WebrtcClient::callAll, "Calls all room clients.")
        .def("call_ids", &WebrtcClient::callIds, "Calls the specified clients.", py::arg("ids"))

        .def("hang_up_all", &WebrtcClient::hangUpAll, "Hangs up all clients.")
        .def(
            "close_all_room_peer_connections",
            &WebrtcClient::closeAllRoomPeerConnections,
            "Closes all room peer connections.")

        .def_property_readonly(
            "is_connected",
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::isConnected),
            "Indicates if the client is connected to the signaling server.\n"
            "\n"
            ":return: True if the client is connected to the signaling server")
        .def_property_readonly(
            "is_rtc_connected",
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::isRtcConnected),
            "Indicates if the client is connected to a least "
            "one client (RTCPeerConnection).\n"
            "\n"
            ":return: True if the client is connected to a "
            "least one client (RTCPeerConnection)")
        .def_property_readonly(
            "id",
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::id),
            "Returns the client id.\n"
            "\n"
            ":return: The client id")

        .def_property_readonly(
            "connected_room_client_ids",
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::getConnectedRoomClientIds),
            "Returns the connected room client ids.\n"
            "\n"
            ":return: The connected room client ids")

        .def(
            "get_room_client",
            &WebrtcClient::getRoomClient,
            py::call_guard<py::gil_scoped_release>(),
            "Returns the room client that matches with the specified id.\n"
            "If no room client matches with the id, a default room client is "
            "returned.\n"
            "\n"
            ":param id: The room client id\n"
            "\n"
            ":return: The room client that matches with the specified id",
            py::arg("id"))
        .def_property_readonly(
            "room_clients",
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::getRoomClients),
            "Returns the room clients\n"
            "\n"
            ":return: The room clients")

        .def_property(
            "on_signaling_connection_opened",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setOnSignalingConnectionOpened),
            "Sets the callback that is called when the signaling "
            "connection opens.\n"
            "\n"
            "The callback is called from the internal client thread. "
            "The callback should not block.\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_signaling_connection_closed",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setOnSignalingConnectionClosed),
            "Sets the callback that is called when the signaling "
            "connection closes.\n"
            "\n"
            "The callback is called from the internal client thread. "
            "The callback should not block.\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_signaling_connection_error",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setOnSignalingConnectionError),
            "Sets the callback that is called when a signaling "
            "connection error occurs.\n"
            "\n"
            "The callback is called from the internal client thread. "
            "The callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - error: The error message\n"
            "\n"
            ":param callback: The callback")

        .def_property(
            "on_room_clients_changed",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setOnRoomClientsChanged),
            "Sets the callback that is called when the room client changes.\n"
            "\n"
            "The callback is called from the internal client thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - room_clients: The room clients\n"
            "\n"
            ":param callback: The callback")

        .def_property(
            "call_acceptor",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setCallAcceptor),
            "Sets the callback that is used to accept or reject a call.\n"
            "\n"
            "The callback is called from the internal client thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client the call is from\n"
            "\n"
            "Callback return value:\n"
            " - True to accept the call, False to reject the call\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_call_rejected",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setOnCallRejected),
            "Sets the callback that is called when a call is rejected.\n"
            "\n"
            "The callback is called from the internal client thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client that rejects the call\n"
            "\n"
            ":param callback: The callback")

        .def_property(
            "on_client_connected",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setOnClientConnected),
            "Sets the callback that is called when a client peer "
            "connection opens.\n"
            "\n"
            "The callback is called from the internal client thread. "
            "The callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client that is connected\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_client_disconnected",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setOnClientDisconnected),
            "Sets the callback that is called when a client peer "
            "connection closes.\n"
            "\n"
            "The callback is called from the internal client thread. "
            "The callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client that is disconnected\n"
            "\n"
            ":param callback: The callback")
        .def_property(
            "on_client_connection_failed",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setOnClientConnectionFailed),
            "Sets the callback that is called when a client peer "
            "connection fails.\n"
            "\n"
            "The callback is called from the internal client thread. "
            "The callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - client: The client that has a connection failure\n"
            "\n"
            ":param callback: The callback")

        .def_property(
            "on_error",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setOnError),
            "Sets the callback that is called when an error occurs.\n"
            "\n"
            "The callback is called from the internal client thread. "
            "The callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - error: The error message\n"
            "\n"
            ":param callback: The callback")

        .def_property(
            "logger",
            nullptr,
            GilScopedRelease<WebrtcClient>::guard(&WebrtcClient::setLogger),
            "Sets the callback that is used to log information.\n"
            "\n"
            "The callback is called from the internal client thread. The "
            "callback should not block.\n"
            "\n"
            "Callback parameters:\n"
            " - message: The message\n"
            "\n"
            ":param callback: The callback")

        .def_property(
            "tls_verification_enabled",
            nullptr,
            &WebrtcClient::setTlsVerificationEnabled,
            "Enable or disable the TLS verification. By default, the "
            "TLS verification is enabled.\n"
            "\n"
            ":param: is_enabled");
}
