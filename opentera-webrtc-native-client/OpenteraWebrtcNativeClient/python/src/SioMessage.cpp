#include <OpenteraWebrtcNativeClientPython/SioMessage.h>

#include <map>

using namespace introlab;
using namespace std;
namespace py = pybind11;

sio::message::ptr introlab::pyObjectToSioMessage(py::object object)
{
    if (py::isinstance<py::bool_>(object))
    {
        return sio::bool_message::create(py::bool_(object).cast<bool>());
    }
    else if (py::isinstance<py::int_>(object))
    {
        return sio::int_message::create(py::int_(object).cast<int64_t>());
    }
    else if (py::isinstance<py::float_>(object))
    {
        return sio::double_message::create(py::float_(object).cast<double>());
    }
    else if (py::isinstance<py::bytes>(object))
    {
        return sio::binary_message::create(make_shared<const string>(py::bytes(object).cast<string>()));
    }
    else if (py::isinstance<py::str>(object))
    {
        return sio::string_message::create(py::str(object).cast<string>());
    }
    else if (py::isinstance<py::list>(object))
    {
        auto message = sio::array_message::create();
        for (const auto x : py::list(object))
        {
            message->get_vector().push_back(pyObjectToSioMessage(x.cast<py::object>()));
        }
        return message;
    }
    else if (py::isinstance<py::dict>(object))
    {
        auto message = sio::object_message::create();
        for (const auto x : py::dict(object))
        {
            if (py::isinstance<py::str>(x.first))
            {
                message->get_map()[py::str(x.first).cast<string>()] = pyObjectToSioMessage(x.second.cast<py::object>());
            }
            else
            {
                py::pybind11_fail("pyObjectToSioMessage: The dict must have str keys.");
            }
        }
        return message;
    }

    return sio::null_message::create();
}

py::object introlab::sioMessageToPyObject(sio::message::ptr message)
{
    switch (message->get_flag())
    {
        case sio::message::flag_integer:
            return py::int_(message->get_int());

        case sio::message::flag_double:
            return py::float_(message->get_double());

        case sio::message::flag_string:
            return py::str(message->get_string());

        case sio::message::flag_binary:
            return py::bytes(*message->get_binary());

        case sio::message::flag_boolean:
            return py::bool_(message->get_bool());

        case sio::message::flag_null:
            return py::none();

        case sio::message::flag_array:
        {
            py::list list;
            for (const auto &x : message->get_vector()) {
                list.append(sioMessageToPyObject(x));
            }
            return list;
        }

        case sio::message::flag_object:
        {
            py::dict dict;
            for (const auto& pair : message->get_map())
            {
                dict[pair.first.c_str()] = sioMessageToPyObject(pair.second);
            }
            return dict;
        }
    }

    return py::none();
}
