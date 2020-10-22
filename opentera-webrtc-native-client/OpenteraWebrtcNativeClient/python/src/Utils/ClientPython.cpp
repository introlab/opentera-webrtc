#include <OpenteraWebrtcNativeClientPython/Utils/ClientPython.h>
#include <OpenteraWebrtcNativeClientPython/SioMessage.h>

#include <OpenteraWebrtcNativeClient/Utils/Client.h>

#include <pybind11/stl.h>

using namespace introlab;
using namespace std;
namespace py = pybind11;

Client clientConstructor(const string& id, const string& name, py::object data)
{
    return Client(id, name, pyObjectToSioMessage(data));
}

RoomClient roomClientConstructor(const string& id, const string& name, py::object data, bool isConnected)
{
    return RoomClient(id, name, pyObjectToSioMessage(data), isConnected);
}

void introlab::initClientPython(pybind11::module& m)
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
