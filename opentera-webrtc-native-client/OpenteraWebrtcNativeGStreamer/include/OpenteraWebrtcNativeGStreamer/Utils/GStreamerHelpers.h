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

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_GSTREAMER_HELPERS_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_GSTREAMER_HELPERS_H

#include <gst/gst.h>
#include <gst/video/video.h>

#include <memory>
#include <type_traits>

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
            gst_message_unref(element);
        }
        template<>
        inline void gst_deleter<GstBus>(GstBus* element)
        {
            gst_object_unref(element);
        }
        template<>
        inline void gst_deleter<GstElement>(GstElement* element)
        {
            gst_object_unref(element);
        }
        template<>
        inline void gst_deleter<GstElementFactory>(GstElementFactory* element)
        {
            gst_object_unref(element);
        }
        template<>
        inline void gst_deleter<GstPipeline>(GstPipeline* element)
        {
            gst_element_set_state(GST_ELEMENT(element), GST_STATE_NULL);
            gst_object_unref(element);
        }
        template<>
        inline void gst_deleter<GstCaps>(GstCaps* element)
        {
            gst_caps_unref(element);
        }

        template<>
        inline void gst_deleter<GstPad>(GstPad* element)
        {
            gst_object_unref(element);
        }
        template<>
        inline void gst_deleter<GstQuery>(GstQuery* element)
        {
            gst_query_unref(element);
        }
        template<>
        inline void gst_deleter<gchar>(gchar* element)
        {
            g_free(element);
        }
        template<>
        inline void gst_deleter<GError>(GError* element)
        {
            g_clear_error(&element);
        }
        template<>
        inline void gst_deleter<GstSample>(GstSample* element)
        {
            gst_sample_unref(element);
        }
        template<>
        inline void gst_deleter<GstBuffer>(GstBuffer* element)
        {
            gst_buffer_unref(element);
        }
        template<>
        inline void gst_deleter<GstBufferPool>(GstBufferPool* element)
        {
            gst_object_unref(element);
        }
        template<>
        inline void gst_deleter<GstVideoConverter>(GstVideoConverter* element)
        {
            gst_video_converter_free(element);
        }
        template<>
        inline void gst_deleter<GstVideoInfo>(GstVideoInfo* element)
        {
            gst_video_info_free(element);
        }
        template<>
        inline void gst_deleter<GstStructure>(GstStructure* element)
        {
            gst_structure_free(element);
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

#endif
