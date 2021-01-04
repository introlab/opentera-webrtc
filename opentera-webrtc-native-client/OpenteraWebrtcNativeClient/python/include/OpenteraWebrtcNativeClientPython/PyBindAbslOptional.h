#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PY_BIND_ABLS_OPTIONAL_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PY_BIND_ABLS_OPTIONAL_H

#include <absl/types/optional.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace pybind11
{
    namespace detail
    {
        template<typename T>
        struct type_caster<absl::optional<T>> : public optional_caster<absl::optional<T>>
        {
        };
    }
}

#endif
