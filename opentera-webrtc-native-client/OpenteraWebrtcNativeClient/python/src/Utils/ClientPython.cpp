#include <OpenteraWebrtcNativeClientPython/Utils/ClientPython.h>
#include <OpenteraWebrtcNativeClientPython/SioMessage.h>

#include <OpenteraWebrtcNativeClient/Utils/Client.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

Client clientConstructor(string id, string name, const py::object& data)
{
    return Client(move(id), move(name), pyObjectToSioMessage(data));
}

RoomClient roomClientConstructor(string id, string name, const py::object& data, bool isConnected)
{
    return RoomClient(move(id), move(name), pyObjectToSioMessage(data), isConnected);
}

void opentera::initClientPython(pybind11::module& m)
{
    py::class_<Client>(m, "Client")
            .def(py::init<>())
            .def(py::init(&clientConstructor), py::arg("id"), py::arg("name"), py::arg("data"))

            .def_property_readonly("id", &Client::id)
            .def_property_readonly("name", &Client::name)
            .def_property_readonly("data", [](const Client& self) { return sioMessageToPyObject(self.data()); });

    py::class_<RoomClient>(m, "RoomClient")
            .def(py::init<>())
            .def(py::init(&roomClientConstructor),
                    py::arg("id"), py::arg("name"), py::arg("data"), py::arg("is_connected"))
            .def(py::init<const Client&, bool>(), py::arg("client"), py::arg("is_connected"))

            .def_property_readonly("id", &RoomClient::id)
            .def_property_readonly("name", &RoomClient::name)
            .def_property_readonly("data", [](const RoomClient& self) { return sioMessageToPyObject(self.data()); })
            .def_property_readonly("is_connected", &RoomClient::isConnected);
}
