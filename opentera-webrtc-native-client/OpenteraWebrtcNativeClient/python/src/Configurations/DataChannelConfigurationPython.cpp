#include <OpenteraWebrtcNativeClientPython/Configurations/DataChannelConfigurationPython.h>

#include <OpenteraWebrtcNativeClient/Configurations/DataChannelConfiguration.h>

#include <pybind11/stl.h>

using namespace introlab;
using namespace std;
namespace py = pybind11;

namespace pybind11
{
    namespace detail
    {
        template<typename T>
        struct type_caster<absl::optional<T>> : public optional_caster<absl::optional<T>>
        {
        };
    }
}

void introlab::initDataChannelConfigurationPython(py::module& m)
{
    py::class_<DataChannelConfiguration>(m, "DataChannelConfiguration")
            .def_static("create", py::overload_cast<>(&DataChannelConfiguration::create))
            .def_static("create", py::overload_cast<bool>(&DataChannelConfiguration::create), py::arg("ordered"))
            .def_static("createProtocol", &DataChannelConfiguration::createProtocol, py::arg("protocol"))
            .def_static("create", py::overload_cast<bool, const string&>(&DataChannelConfiguration::create),
                    py::arg("ordered"), py::arg("protocol"))

            .def_static("create_max_packet_life_time",
                    py::overload_cast<int>(&DataChannelConfiguration::createMaxPacketLifeTime),
                    py::arg("max_packet_life_time"))
            .def_static("create_max_packet_life_time",
                    py::overload_cast<bool, int>(&DataChannelConfiguration::createMaxPacketLifeTime),
                    py::arg("ordered"), py::arg("max_packet_life_time"))
            .def_static("create_max_packet_life_time",
                    py::overload_cast<int, const string&>(&DataChannelConfiguration::createMaxPacketLifeTime),
                    py::arg("max_packet_life_time"), py::arg("protocol"))
            .def_static("create_max_packet_life_time",
                    py::overload_cast<bool, int, const string&>(&DataChannelConfiguration::createMaxPacketLifeTime),
                    py::arg("ordered"), py::arg("max_packet_life_time"), py::arg("protocol"))

            .def_static("create_max_retransmits",
                    py::overload_cast<int>(&DataChannelConfiguration::createMaxRetransmits), py::arg("max_retransmits"))
            .def_static("create_max_retransmits",
                    py::overload_cast<bool, int>(&DataChannelConfiguration::createMaxRetransmits),
                    py::arg("ordered"), py::arg("max_retransmits"))
            .def_static("create_max_retransmits",
                    py::overload_cast<int, const string&>(&DataChannelConfiguration::createMaxRetransmits),
                    py::arg("max_retransmits"), py::arg("protocol"))
            .def_static("create_max_retransmits",
                    py::overload_cast<bool, int, const string&>(&DataChannelConfiguration::createMaxRetransmits),
                    py::arg("ordered"), py::arg("max_retransmits"), py::arg("protocol"))

            .def_property_readonly("ordered", &DataChannelConfiguration::ordered)
            .def_property_readonly("max_packet_life_time", &DataChannelConfiguration::maxPacketLifeTime)
            .def_property_readonly("max_retransmits", &DataChannelConfiguration::maxRetransmits)
            .def_property_readonly("protocol", &DataChannelConfiguration::protocol);
}
