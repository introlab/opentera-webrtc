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

#pragma once

#include <memory>
#include <type_traits>
#include <iostream>

#include <gst/gst.h>
#include <gst/video/video.h>

namespace gst
{
    namespace internal
    {
        // Explicit instanciation of the template is done in the following section for any
        // Gst type that we wish to delete
        // Everything else won't compile, so that we can catch undefined deleters and add
        // them to this list
        // This templated function is then used by a class gst::gst_delete, which is
        // implemented based on the standard std::default_delete
        template<typename T>
        inline void gst_deleter(T*) = delete;

        template<>
        inline void gst_deleter<GstMessage>(GstMessage* element)
        {
            std::cout << "Delete message\n";
            gst_message_unref(element);
        }
        template<>
        inline void gst_deleter<GstBus>(GstBus* element)
        {
            std::cout << "Delete bus\n";
            gst_object_unref(element);
        }
        template<>
        inline void gst_deleter<GstElement>(GstElement* element)
        {
            std::cout << "Delete element\n";
            gst_object_unref(element);
        }
        template<>
        inline void gst_deleter<GstPipeline>(GstPipeline* element)
        {
            std::cout << "Set pipeline to null state and delete\n";
            gst_element_set_state(GST_ELEMENT(element), GST_STATE_NULL);
            gst_object_unref(element);
        }
        template<>
        inline void gst_deleter<GstCaps>(GstCaps* element)
        {
            std::cout << "Delete caps\n";
            gst_caps_unref(element);
        }

        template<>
        inline void gst_deleter<GstPad>(GstPad* element)
        {
            std::cout << "Delete pad\n";
            gst_object_unref(element);
        }
        template<>
        inline void gst_deleter<GstQuery>(GstQuery* element)
        {
            std::cout << "Delete query\n";
            gst_query_unref(element);
        }
        template<>
        inline void gst_deleter<gchar>(gchar* element)
        {
            std::cout << "Delete gchar*\n";
            g_free(element);
        }
        template<>
        inline void gst_deleter<GError>(GError* element)
        {
            std::cout << "Delete error\n";
            g_clear_error(&element);
        }
        template<>
        inline void gst_deleter<GstSample>(GstSample* element)
        {
            std::cout << "Delete sample " << std::hex << element << std::dec << "\n";
            gst_sample_unref(element);
        }
        template<>
        inline void gst_deleter<GstBuffer>(GstBuffer* element)
        {
            std::cout << "Delete buffer\n";
            gst_buffer_unref(element);
        }
        template<>
        inline void gst_deleter<GstVideoConverter>(GstVideoConverter* element)
        {
            std::cout << "Delete video converter\n";
            gst_video_converter_free(element);
        }


        // Use to remove required static_cast when using some GStreamer enums as
        // bitmasks/flags
        // Overloads of operator| are provided outside in the global namespace, forwarding
        // computation to this implementation
        template<typename T>
        inline T bitmask_or(T a, T b)
        {
            return static_cast<T>(static_cast<int>(a) | static_cast<int>(b));
        }

    }  // namespace internal

    // RAII wrapper for GStreamer initialization/deinitialization
    class Gst
    {
    public:
        explicit Gst(int* argc, char** argv[])
        {
            if (!gst_is_initialized())
            {
                gst_init(argc, argv);
            }
        }
        ~Gst()
        {
            if (gst_is_initialized())
            {
                gst_deinit();
            }
        }
    };

    // Interface inspired from std::default_delete
    // (cppreference: https://en.cppreference.com/w/cpp/memory/default_delete)
    template<typename T>
    struct gst_delete
    {
        void operator()(T* element) const { gst::internal::gst_deleter<T>(element); }
    };

    // Our own unique_ptr specialization that uses gst_delete as deleter
    template<typename T>
    using unique_ptr = std::unique_ptr<T, gst::gst_delete<T>>;

    // Easily create a gst::unique_ptr from a raw pointer using TAD, while
    // default-constructing the deleter
    template<typename T>
    inline auto unique_from_ptr(T* ptr)
    {
        return gst::unique_ptr<T>(ptr);
    }

    // Easily create a std::shared_ptr from a raw pointer using TAD, using a gst::gst_delete deleter
    template<typename T>
    inline auto shared_from_ptr(T* ptr)
    {
        return std::shared_ptr<T>(ptr, gst::gst_delete<T>{});
    }

    // RAII wrapper for request pads, which need a reference to the element from which
    // they come for cleanup
    class GstRequestPad
    {
    public:
        GstRequestPad(GstElement* element, const gchar* name) noexcept
            : element_{element},
              pad_{gst::unique_from_ptr(gst_element_get_request_pad(element, name))}
        {
        }
        ~GstRequestPad() noexcept
        {
            if (pad_ && element_ != nullptr)
            {
                std::cout << "Release request pad\n";
                gst_element_release_request_pad(GST_ELEMENT(element_), pad_.get());
            }
        }
        operator GstPad*() { return pad_.get(); }
        operator GstObject*() { return GST_OBJECT(pad_.get()); }

    private:
        GstElement* element_;
        gst::unique_ptr<GstPad> pad_;
    };
}  // namespace gst

// Remove required static_cast when using some GStreamer enums as bitmasks/flags
inline GstMessageType operator|(GstMessageType a, GstMessageType b)
{
    return gst::internal::bitmask_or(a, b);
}
inline GstSeekFlags operator|(GstSeekFlags a, GstSeekFlags b)
{
    return gst::internal::bitmask_or(a, b);
}
inline GstMemoryFlags operator|(GstMemoryFlags a, GstMemoryFlags b)
{
    return gst::internal::bitmask_or(a, b);
}
