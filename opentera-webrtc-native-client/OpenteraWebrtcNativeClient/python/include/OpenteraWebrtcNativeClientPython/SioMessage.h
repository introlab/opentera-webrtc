#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_SIO_MESSAGE_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_SIO_MESSAGE_H

#include <sio_message.h>

#include <pybind11/pybind11.h>

namespace opentera
{
    PYBIND11_EXPORT sio::message::ptr pyObjectToSioMessage(const pybind11::object& message);
    PYBIND11_EXPORT pybind11::object sioMessageToPyObject(sio::message::ptr message);
}

#endif
