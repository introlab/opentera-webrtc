#include <OpenteraWebrtcNativeClientPython/Utils/IceServerPython.h>

#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

#include <pybind11/stl.h>

using namespace introlab;
using namespace std;
namespace py = pybind11;

void introlab::initIceServerPython(pybind11::module& m)
{
    py::class_<IceServer>(m, "IceServer")
            .def(py::init<const string&>(), py::arg("url"))
            .def(py::init<const string&, const string&, const string&>(),
                    py::arg("url"), py::arg("username"), py::arg("credential"))
            .def(py::init<const vector<string>&>(), py::arg("urls"))
            .def(py::init<const vector<string>&, const string&, const string&>(),
                    py::arg("urls"), py::arg("username"), py::arg("credential"))

            .def_property_readonly("urls", &IceServer::urls)
            .def_property_readonly("username", &IceServer::username)
            .def_property_readonly("credential", &IceServer::credential);
}
