#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_SIO_MESSAGE_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_SIO_MESSAGE_H

#include <nlohmann/json.hpp>

#include <pybind11/pybind11.h>

namespace opentera
{
    PYBIND11_EXPORT nlohmann::json pyObjectToJson(const pybind11::object& message);
    PYBIND11_EXPORT pybind11::object jsonToPyObject(const nlohmann::json& json);
}

#endif
