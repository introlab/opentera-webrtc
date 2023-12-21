#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_WEBRTC_CLIENT_PYTHON_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_WEBRTC_CLIENT_PYTHON_H

#include <pybind11/pybind11.h>

namespace opentera
{
    PYBIND11_EXPORT void initWebrtcClientPython(pybind11::module& m);
}

#endif
