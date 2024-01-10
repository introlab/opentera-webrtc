#include <OpenteraWebrtcNativeClientPython/Json.h>
#include <OpenteraWebrtcNativeClientPython/Utils/ClientPython.h>

#include <OpenteraWebrtcNativeClient/Utils/Client.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

Client clientConstructor(string id, string name, const py::object& data)
{
    return Client(move(id), move(name), pyObjectToJson(data));
}

RoomClient roomClientConstructor(string id, string name, const py::object& data, bool isConnected)
{
    return RoomClient(move(id), move(name), pyObjectToJson(data), isConnected);
}

void opentera::initClientPython(pybind11::module& m)
{
    py::class_<Client>(m, "Client", "Represents a peer client.")
        .def(py::init<>(), "Creates a client with default values")
        .def(
            py::init(&clientConstructor),
            "Creates a client with the specified values.\n"
            "\n"
            ":param id: The client id\n"
            ":param name: The client name\n"
            ":param data: The client data",
            py::arg("id"),
            py::arg("name"),
            py::arg("data"))

        .def_property_readonly(
            "id",
            &Client::id,
            "Returns the client id.\n"
            "\n"
            ":return: The client id")
        .def_property_readonly(
            "name",
            &Client::name,
            "Returns the client name.\n"
            "\n"
            ":return: The client name")
        .def_property_readonly(
            "data",
            [](const Client& self) { return jsonToPyObject(self.data()); },
            "Returns the client data.\n"
            "\n"
            ":return: The client data");

    py::class_<RoomClient>(m, "RoomClient")
        .def(py::init<>(), "Creates a room client with default values")
        .def(
            py::init(&roomClientConstructor),
            "Creates a room client with the specified values.\n"
            "\n"
            ":param id: The client id\n"
            ":param name: The client name\n"
            ":param data: The client data\n"
            ":param is_connected: Indicates if the client is connected "
            "(RTCPeerConnection)",
            py::arg("id"),
            py::arg("name"),
            py::arg("data"),
            py::arg("is_connected"))
        .def(
            py::init<const Client&, bool>(),
            "Creates a room client from a client.\n"
            "\n"
            ":param client: The client\n"
            ":param is_connected: Indicates if the client is connected "
            "(RTCPeerConnection)",
            py::arg("client"),
            py::arg("is_connected"))

        .def_property_readonly(
            "id",
            &RoomClient::id,
            "Returns the client id.\n"
            "\n"
            ":return: The client id")
        .def_property_readonly(
            "name",
            &RoomClient::name,
            "Returns the client name.\n"
            "\n"
            ":return: The client name")
        .def_property_readonly(
            "data",
            [](const RoomClient& self) { return jsonToPyObject(self.data()); },
            "Returns the client data.\n"
            "\n"
            ":return: The client data")
        .def_property_readonly(
            "is_connected",
            &RoomClient::isConnected,
            "Indicates if the client is connected (RTCPeerConnection).\n"
            "\n"
            ":return: True if the client is connected (RTCPeerConnection)");
}
