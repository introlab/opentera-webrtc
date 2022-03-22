#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PY_BIND_UTILS_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PY_BIND_UTILS_H

#include <pybind11/pybind11.h>

#include <functional>

template<class type_>
struct GilScopedRelease
{
    template<class F>
    static pybind11::cpp_function guard(const F& f)
    {
        return pybind11::cpp_function(
            pybind11::method_adaptor<type_>(f),
            pybind11::call_guard<pybind11::gil_scoped_release>());
    }
};

#endif
