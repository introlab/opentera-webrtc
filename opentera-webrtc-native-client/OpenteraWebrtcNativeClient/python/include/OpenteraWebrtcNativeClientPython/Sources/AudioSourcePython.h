#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_SOURCES_AUDIO_SOURCE_PYTHON_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_SOURCES_AUDIO_SOURCE_PYTHON_H

#include <pybind11/pybind11.h>

namespace opentera
{
    PYBIND11_EXPORT void initAudioSourcePython(pybind11::module& m);
}

#endif
