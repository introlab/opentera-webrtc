#include <OpenteraWebrtcNativeClientPython/Utils/IceServerPython.h>

#include <OpenteraWebrtcNativeClient/Utils/IceServer.h>

#include <pybind11/stl.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initIceServerPython(pybind11::module& m)
{
    py::class_<IceServer>(m, "IceServer")
            .def(py::init<string>(), py::arg("url"))
            .def(py::init<string, string, string>(),
                    py::arg("url"), py::arg("username"), py::arg("credential"))
            .def(py::init<vector<string>>(), py::arg("urls"))
            .def(py::init<vector<string>, string, string>(),
                    py::arg("urls"), py::arg("username"), py::arg("credential"))

            .def_property_readonly("urls", &IceServer::urls)
            .def_property_readonly("username", &IceServer::username)
            .def_property_readonly("credential", &IceServer::credential)

            .def_static("fetch_from_server", [](const string& url, const string& password)
            {
                vector<IceServer> iceServers;
                if (IceServer::fetchFromServer(url, password, iceServers))
                {
                    return iceServers;
                }
                else
                {
                    throw runtime_error("\"fetch_from_server\" failed");
                }
            }, py::arg("url"), py::arg("password"))
            .def_static("from_json", [](const string& json)
            {
                vector<IceServer> iceServers;
                if (IceServer::fromJson(json, iceServers))
                {
                    return iceServers;
                }
                else
                {
                    throw py::value_error("Invalid json");
                }
            }, py::arg("json"));
}
