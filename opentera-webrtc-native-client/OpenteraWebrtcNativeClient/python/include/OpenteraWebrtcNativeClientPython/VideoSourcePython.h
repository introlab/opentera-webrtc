#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_VIDEO_SOURCE_PYTHON_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_VIDEO_SOURCE_PYTHON_H

#include <pybind11/pybind11.h>

namespace introlab
{
    PYBIND11_EXPORT void initVideoSourcePython(pybind11::module& m);
}

#endif
