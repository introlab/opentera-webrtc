#include <OpenteraWebrtcNativeClientPython/Configurations/DataChannelConfigurationPython.h>

#include <OpenteraWebrtcNativeClient/Configurations/DataChannelConfiguration.h>

#include <pybind11/stl.h>

using namespace opentera;
using namespace std;
namespace py = pybind11;

void opentera::initDataChannelConfigurationPython(py::module& m)
{
    py::class_<DataChannelConfiguration>(m, "DataChannelConfiguration", "Represents a data channel configuration")
        .def_static(
            "create",
            py::overload_cast<>(&DataChannelConfiguration::create),
            "Creates a data channel configuration with default values.\n"
            "\n"
            ":return: A data channel configuration with default values")
        .def_static(
            "create",
            py::overload_cast<bool>(&DataChannelConfiguration::create),
            py::arg("ordered"),
            "Creates a data channel configuration with the specified value.\n"
            "\n"
            ":param ordered: Indicates if the message order must be preserved\n"
            "\n"
            ":return: A data channel configuration with the specified value")
        .def_static(
            "create_protocol",
            &DataChannelConfiguration::createProtocol,
            py::arg("protocol"),
            "Creates a data channel configuration with the specified value.\n"
            "\n"
            ":param protocol: The data channel protocol\n"
            "\n"
            ":return: A data channel configuration with the specified value")
        .def_static(
            "create",
            py::overload_cast<bool, string>(&DataChannelConfiguration::create),
            "Creates a data channel configuration with the specified values.\n"
            "\n"
            ":param ordered: Indicates if the message order must be preserved\n"
            ":param protocol: The data channel protocol\n"
            "\n"
            ":return: A data channel configuration with the specified values",
            py::arg("ordered"),
            py::arg("protocol"))

        .def_static(
            "create_max_packet_life_time",
            py::overload_cast<int>(&DataChannelConfiguration::createMaxPacketLifeTime),
            "Creates a data channel configuration with the specified value.\n"
            "\n"
            ":param max_packet_life_time: Indicates the amount of time a message "
            "can be retransmitted (ms)\n"
            "\n"
            ":return: A data channel configuration with the specified value",
            py::arg("max_packet_life_time"))
        .def_static(
            "create_max_packet_life_time",
            py::overload_cast<bool, int>(&DataChannelConfiguration::createMaxPacketLifeTime),
            "Creates a data channel configuration with the specified values.\n"
            "\n"
            ":param ordered: Indicates if the message order must be preserved\n"
            ":param max_packet_life_time: Indicates the amount of time a message "
            "can be retransmitted (ms)\n"
            "\n"
            ":return: A data channel configuration with the specified values",
            py::arg("ordered"),
            py::arg("max_packet_life_time"))
        .def_static(
            "create_max_packet_life_time",
            py::overload_cast<int, string>(&DataChannelConfiguration::createMaxPacketLifeTime),
            "Creates a data channel configuration with the specified values.\n"
            "\n"
            ":param max_packet_life_time: Indicates the amount of time a message "
            "can be retransmitted (ms)\n"
            ":param protocol: The data channel protocol\n"
            "\n"
            ":return: A data channel configuration with the specified values",
            py::arg("max_packet_life_time"),
            py::arg("protocol"))
        .def_static(
            "create_max_packet_life_time",
            py::overload_cast<bool, int, string>(&DataChannelConfiguration::createMaxPacketLifeTime),
            "Creates a data channel configuration with the specified values.\n"
            "\n"
            ":param ordered: Indicates if the message order must be preserved\n"
            ":param max_packet_life_time: Indicates the amount of time a message "
            "can be retransmitted (ms)\n"
            ":param protocol: The data channel protocol\n"
            "\n"
            ":return: A data channel configuration with the specified values",
            py::arg("ordered"),
            py::arg("max_packet_life_time"),
            py::arg("protocol"))

        .def_static(
            "create_max_retransmits",
            py::overload_cast<int>(&DataChannelConfiguration::createMaxRetransmits),
            "Creates a data channel configuration with the specified value.\n"
            "\n"
            ":param max_retransmits: Indicates the maximum number of time a "
            "message can be retransmitted\n"
            "\n"
            ":return: A data channel configuration with the specified value",
            py::arg("max_retransmits"))
        .def_static(
            "create_max_retransmits",
            py::overload_cast<bool, int>(&DataChannelConfiguration::createMaxRetransmits),
            "Creates a data channel configuration with the specified values.\n"
            "\n"
            ":param ordered: Indicates if the message order must be preserved\n"
            ":param max_retransmits: Indicates the maximum number of time a "
            "message can be retransmitted\n"
            "\n"
            ":return: A data channel configuration with the specified values",
            py::arg("ordered"),
            py::arg("max_retransmits"))
        .def_static(
            "create_max_retransmits",
            py::overload_cast<int, string>(&DataChannelConfiguration::createMaxRetransmits),
            "Creates a data channel configuration with the specified values.\n"
            "\n"
            ":param max_retransmits: Indicates the maximum number of time a "
            "message can be retransmitted\n"
            ":param protocol: The data channel protocol\n"
            "\n"
            ":return: A data channel configuration with the specified values",
            py::arg("max_retransmits"),
            py::arg("protocol"))
        .def_static(
            "create_max_retransmits",
            py::overload_cast<bool, int, string>(&DataChannelConfiguration::createMaxRetransmits),
            "Creates a data channel configuration with the specified values.\n"
            "\n"
            ":param ordered: Indicates if the message order must be preserved\n"
            ":param max_retransmits: Indicates the maximum number of time a "
            "message can be retransmitted\n"
            ":param protocol: The data channel protocol\n"
            "\n"
            ":return: A data channel configuration with the specified values",
            py::arg("ordered"),
            py::arg("max_retransmits"),
            py::arg("protocol"))

        .def_property_readonly(
            "ordered",
            &DataChannelConfiguration::ordered,
            "Indicates if the message order must be preserved.\n"
            "\n"
            ":return: True if the message order must be preserved")
        .def_property_readonly(
            "max_packet_life_time",
            &DataChannelConfiguration::maxPacketLifeTime,
            "Returns the maximum number of time a message can be retransmitted.\n"
            "\n"
            ":return: The maximum number of time a message can be retransmitted")
        .def_property_readonly(
            "max_retransmits",
            &DataChannelConfiguration::maxRetransmits,
            "Returns the maximum number of time a message can be retransmitted.\n"
            "\n"
            ":return: The maximum number of time a message can be retransmitted")
        .def_property_readonly(
            "protocol",
            &DataChannelConfiguration::protocol,
            "Returns the data channel protocol.\n"
            "\n"
            ":return: The data channel protocol");
}
