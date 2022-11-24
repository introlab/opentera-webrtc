#pragma once

#include <memory>
#include <type_traits>
extern "C"
{
#include <gst/gst.h>
#include <gst/video/video.h>
}
#include <iostream>

namespace gst
{
    namespace internal
    {
        // Explicit instanciation of the template is done in the following section for any
        // Gst type that we wish to delete
        // Everything else won't compile, so that we can catch undefined deleters and add
        // them to this list
        // This templated functiom is then used by a class gst::gst_delete, which is
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
            std::cout << "Set element to null state and delete\n";
            gst_element_set_state(element, GST_STATE_NULL);
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
            std::cout << "Delete sample\n";
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

    }

    // Inspired from std::out_ptr_t in the C++23 standard
    // (cppreference: https://en.cppreference.com/w/cpp/memory/out_ptr_t)
    // (MSVC implementation:
    // https://github.com/microsoft/STL/blob/2f03bdf361f7f153b4216c60a0d9491c0be13a73/stl/inc/memory)
    // (Pete Woods/Canonical implementation for GStreamer for Unity:
    // https://github.com/pete-woods/unity-api/blob/master/include/unity/util/GlibMemory.h)
    template<typename Smart, typename P>
    class out_ptr_t
    {
    public:
        using ElementType = std::remove_pointer_t<P>;

        explicit out_ptr_t(Smart& s) noexcept : smart_(s) {}

        out_ptr_t(const out_ptr_t&) = delete;
        out_ptr_t& operator=(const out_ptr_t&) = delete;

        out_ptr_t(out_ptr_t&& other) noexcept : smart_(other.smart_), ptr_(other.ptr_)
        {
            if (this != &other)
            {
                other.ptr_ = nullptr;
            }
        }
        out_ptr_t& operator=(out_ptr_t&& other) noexcept
        {
            if (this != &other)
            {
                smart_ = other.smart_;
                ptr_ = other.ptr_;
                other.ptr_ = nullptr;
            }
            return *this;
        }
        ~out_ptr_t() noexcept { smart_.reset(ptr_); }

        operator P*() noexcept { return std::addressof(ptr_); }

    private:
        Smart& smart_;
        P ptr_ = nullptr;
    };

    // Inspired from std::out_ptr in the C++23 standard
    // (cppreference: https://en.cppreference.com/w/cpp/memory/out_ptr_t/out_ptr)
    template<typename Pointer = void, typename Smart>
    inline auto out_ptr(Smart& s)
    {
        std::cout << "Creating out_ptr\n";
        if constexpr (!std::is_void_v<Pointer>)
        {
            std::cout << "Using Pointer\n";
            return out_ptr_t<Smart, Pointer>(s);
        }
        else
        {
            std::cout << "Using Smart::pointer\n";
            return out_ptr_t<Smart, typename Smart::pointer>(s);
        }
    }

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
    class gst_delete
    {
    public:
        constexpr gst_delete() noexcept = default;

        template<typename U, typename = std::enable_if_t<std::is_convertible<U*, T*>::value>>
        gst_delete(const gst_delete<U>& d) noexcept : gst_delete(d)
        {
        }

        template<typename U, typename = std::enable_if_t<std::is_convertible<U (*)[], T (*)[]>::value>>
        gst_delete(const gst_delete<U[]>& d) noexcept : gst_delete(d)
        {
        }

        void operator()(T* element) const { gst::internal::gst_deleter<T>(element); }
    };

    // Our own unique_ptr specialization that uses gst_delete as deleter
    template<typename T>
    using unique_ptr = std::unique_ptr<T, gst::gst_delete<T>>;
    // // Easily create a gst::unique_ptr from a raw pointer using TAD, while
    // default-constructing the deleter
    template<typename T>
    inline auto unique_from_ptr(T* ptr)
    {
        return gst::unique_ptr<T>(ptr);
    }
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
}

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
