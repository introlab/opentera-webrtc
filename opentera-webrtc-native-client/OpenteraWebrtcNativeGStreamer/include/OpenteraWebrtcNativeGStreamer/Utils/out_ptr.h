/*
 *  Copyright 2022 IntRoLab
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_OUT_PTR_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_OUT_PTR_H

#include <memory>
#include <iostream>

namespace opentera
{
    // Inspired from std::out_ptr_t in the C++23 standard
    // (cppreference: https://en.cppreference.com/w/cpp/memory/out_ptr_t)
    // (MSVC implementation:
    // https://github.com/microsoft/STL/blob/2f03bdf361f7f153b4216c60a0d9491c0be13a73/stl/inc/memory)
    template<typename Smart, typename Ptr>
    class out_ptr_t
    {
    public:
        explicit out_ptr_t(Smart& s) noexcept : m_smart(s) {}

        out_ptr_t(const out_ptr_t&) = delete;
        out_ptr_t& operator=(const out_ptr_t&) = delete;

        ~out_ptr_t() noexcept { m_smart.reset(m_ptr); }

        operator Ptr*() noexcept { return std::addressof(m_ptr); }

    private:
        Smart& m_smart;
        Ptr m_ptr = nullptr;
    };

    // Inspired from std::out_ptr in the C++23 standard
    // (cppreference: https://en.cppreference.com/w/cpp/memory/out_ptr_t/out_ptr)
    template<typename Pointer = void, typename Smart>
    inline auto out_ptr(Smart& s)
    {
        if constexpr (!std::is_void_v<Pointer>)
        {
            return out_ptr_t<Smart, Pointer>(s);
        }
        else
        {
            return out_ptr_t<Smart, typename Smart::pointer>(s);
        }
    }
}

#endif
