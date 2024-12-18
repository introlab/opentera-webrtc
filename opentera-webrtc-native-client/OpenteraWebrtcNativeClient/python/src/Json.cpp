#include <OpenteraWebrtcNativeClientPython/Json.h>

#include <map>

using namespace opentera;
using namespace std;
namespace py = pybind11;

nlohmann::json opentera::pyObjectToJson(const py::object& object)
{
    if (py::isinstance<py::bool_>(object))
    {
        return py::bool_(object).cast<bool>();
    }
    else if (py::isinstance<py::int_>(object))
    {
        return py::int_(object).cast<int64_t>();
    }
    else if (py::isinstance<py::float_>(object))
    {
        return py::float_(object).cast<double>();
    }
    else if (py::isinstance<py::bytes>(object))
    {
        auto dataStr = py::bytes(object).cast<string>();
        vector<uint8_t> data;
        copy(dataStr.begin(), dataStr.end(), back_inserter(data));
        return nlohmann::json::binary(data);
    }
    else if (py::isinstance<py::str>(object))
    {
        return py::str(object).cast<string>();
    }
    else if (py::isinstance<py::list>(object))
    {
        auto json = nlohmann::json::array();
        for (const auto x : py::list(object))
        {
            json.emplace_back(pyObjectToJson(x.cast<py::object>()));
        }
        return json;
    }
    else if (py::isinstance<py::dict>(object))
    {
        auto json = nlohmann::json::object();
        for (const auto x : py::dict(object))
        {
            if (py::isinstance<py::str>(x.first))
            {
                json.emplace(py::str(x.first).cast<string>(), pyObjectToJson(x.second.cast<py::object>()));
            }
            else
            {
                py::pybind11_fail("pyObjectToSioMessage: The dict must have str keys.");
            }
        }
        return json;
    }

    return {};
}

py::object opentera::jsonToPyObject(const nlohmann::json& json)
{
    switch (json.type())
    {
        case nlohmann::detail::value_t::number_integer:
            return py::int_(static_cast<int64_t>(json));

        case nlohmann::detail::value_t::number_unsigned:
            return py::int_(static_cast<uint64_t>(json));

        case nlohmann::detail::value_t::number_float:
            return py::float_(static_cast<double>(json));

        case nlohmann::detail::value_t::string:
            return py::str(json);

        case nlohmann::detail::value_t::binary:
        {
            nlohmann::json::binary_t data = json;
            return py::bytes(string(data.begin(), data.end()));
        }

        case nlohmann::detail::value_t::boolean:
            return py::bool_(json);

        case nlohmann::detail::value_t::null:
            return py::none();

        case nlohmann::detail::value_t::array:
        {
            py::list list;
            for (const auto& x : json)
            {
                list.append(jsonToPyObject(x));
            }
            return list;
        }

        case nlohmann::detail::value_t::object:
        {
            py::dict dict;
            for (const auto& pair : json.items())
            {
                dict[pair.key().c_str()] = jsonToPyObject(pair.value());
            }
            return dict;
        }

        case nlohmann::detail::value_t::discarded:
            return py::none();
    }

    return py::none();
}
