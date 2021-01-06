#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_PYTHON_VIDEO_SOURCE_PYTHON_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_PYTHON_VIDEO_SOURCE_PYTHON_H

#include <pybind11/pybind11.h>

namespace opentera
{
    PYBIND11_EXPORT void initVideoSourcePython(pybind11::module& m);
}

#endif
