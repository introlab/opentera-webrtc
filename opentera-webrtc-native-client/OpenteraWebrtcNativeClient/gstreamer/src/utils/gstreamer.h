#pragma once

#include <gst/gst.h>
#include <string_view>
#include <memory>
// #include "gstreamer_helpers.h"
#include "OpenteraWebrtcNativeClient/Utils/ClassMacro.h"

namespace gst
{
    namespace internal
    {
        // Use to remove required static_cast when using some GStreamer enums as
        // bitmasks/flags
        // Overloads of operator| are provided outside in the global namespace, forwarding
        // computation to this implementation
        template<typename T>
        inline T bitmask_or(T a, T b)
        {
            return static_cast<T>(static_cast<int>(a) | static_cast<int>(b));
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
        auto out_ptr(Smart& s)
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
    }

    // RAII wrapper for GStreamer initialization/deinitialization
    class Gst
    {
    public:
        explicit Gst(int* argc, char** argv[]) { gst_init(argc, argv); }
        ~Gst() { gst_deinit(); }
        DECLARE_NOT_COPYABLE(Gst);
        DECLARE_NOT_MOVABLE(Gst);
    };

    // RAII wrapper for GStreamer element creation/destruction
    class Element
    {
    public:
        explicit Element(GstElement* element) : m_element(element) {}
        ~Element() { gst_object_unref(m_element); }
        DECLARE_NOT_COPYABLE(Element);
        DECLARE_NOT_MOVABLE(Element);

        operator GstElement*() { return m_element; }

    private:
        GstElement* m_element;
    };

    // RAII wrapper for GStreamer pipeline creation/destruction
    class Pipeline
    {
    public:
        explicit Pipeline(GstElement* pipeline) : m_pipeline(pipeline) {}
        ~Pipeline()
        {
            gst_element_set_state(m_pipeline, GST_STATE_NULL);
            gst_object_unref(m_pipeline);
        }
        DECLARE_NOT_COPYABLE(Pipeline);
        DECLARE_NOT_MOVABLE(Pipeline);

        operator GstElement*() { return m_pipeline; }

    private:
        GstElement* m_pipeline;
    };

    // RAII wrapper for GStreamer bus creation/destruction
    class Bus
    {
    public:
        explicit Bus(GstBus* bus) : m_bus(bus) {}
        ~Bus() { gst_object_unref(m_bus); }
        DECLARE_NOT_COPYABLE(Bus);
        DECLARE_NOT_MOVABLE(Bus);

        operator GstBus*() { return m_bus; }

    private:
        GstBus* m_bus;
    };

    // RAII wrapper for GStreamer message creation/destruction
    class Message
    {
    public:
        explicit Message(GstMessage* message) : m_message(message) {}
        ~Message() { gst_message_unref(m_message); }
        DECLARE_NOT_COPYABLE(Message);
        DECLARE_NOT_MOVABLE(Message);

        operator GstMessage*() { return m_message; }

    private:
        GstMessage* m_message;
    };

    // RAII wrapper for GStreamer caps creation/destruction
    class Caps
    {
    public:
        explicit Caps(GstCaps* caps) : m_caps(caps) {}
        ~Caps() { gst_caps_unref(m_caps); }
        DECLARE_NOT_COPYABLE(Caps);
        DECLARE_NOT_MOVABLE(Caps);

        operator GstCaps*() { return m_caps; }

    private:
        GstCaps* m_caps;
    };

    // RAII wrapper for GStreamer pad creation/destruction
    class Pad
    {
    public:
        explicit Pad(GstPad* pad) : m_pad(pad) {}
        ~Pad() { gst_object_unref(m_pad); }
        DECLARE_NOT_COPYABLE(Pad);
        DECLARE_NOT_MOVABLE(Pad);

        operator GstPad*() { return m_pad; }

    private:
        GstPad* m_pad;
    };

    // RAII wrapper for GStreamer query creation/destruction
    class Query
    {
    public:
        explicit Query(GstQuery* query) : m_query(query) {}
        ~Query() { gst_query_unref(m_query); }
        DECLARE_NOT_COPYABLE(Query);
        DECLARE_NOT_MOVABLE(Query);

        operator GstQuery*() { return m_query; }

    private:
        GstQuery* m_query;
    };

    // RAII wrapper for GStreamer gchar* creation/destruction
    class gchar
    {
    public:
        explicit gchar(gchar* gchar) : m_gchar(gchar) {}
        ~gchar() { g_free(m_gchar); }
        DECLARE_NOT_COPYABLE(gchar);
        DECLARE_NOT_MOVABLE(gchar);

        operator gchar*() { return m_gchar; }

    private:
        gchar* m_gchar;
    };

    //    // RAII wrapper for GStreamer GstBuffer* creation/destruction
    //    class Buffer
    //    {
    //    public:
    //        explicit Buffer(GstBuffer* buffer) : m_buffer(buffer) {}
    //        ~Buffer() { gst_buffer_unref(m_buffer); }
    //
    //        operator GstBuffer*() { return m_buffer; }
    //    private:
    //        GstBuffer* m_buffer;
    //    };
    //
    //    // RAII wrapper for GStreamer GstSample* creation/destruction
    //    class Sample
    //    {
    //    public:
    //        explicit Sample(GstSample* sample) : m_sample(sample) {}
    //        ~Sample() { gst_sample_unref(m_sample); }
    //
    //        operator GstSample*() { return m_sample; }
    //    private:
    //        GstSample* m_sample;
    //    };
    //
    //    // RAII wrapper for GStreamer GstMapInfo* creation/destruction
    //    class MapInfo
    //    {
    //    public:
    //        explicit MapInfo(GstMapInfo* mapInfo) : m_mapInfo(mapInfo) {}
    //        ~MapInfo() { gst_buffer_unmap(m_mapInfo->buffer, m_mapInfo); }
    //
    //        operator GstMapInfo*() { return m_mapInfo; }
    //    private:
    //        GstMapInfo* m_mapInfo;
    //    };
    //
    //    // RAII wrapper for GStreamer GstStructure* creation/destruction
    //    class Structure
    //    {
    //    public:
    //        explicit Structure(GstStructure* structure) : m_structure(structure) {}
    //        ~Structure() { gst_structure_free(m_structure); }
    //
    //        operator GstStructure*() { return m_structure; }
    //    private:
    //        GstStructure* m_structure;
    //    };
    //
    //    // RAII wrapper for GStreamer GstTagList* creation/destruction
    //    class TagList
    //    {
    //    public:
    //        explicit TagList(GstTagList* tagList) : m_tagList(tagList) {}
    //        ~TagList() { gst_tag_list_unref(m_tagList); }
    //
    //        operator GstTagList*() { return m_tagList; }
    //    private:
    //        GstTagList* m_tagList;
    //    };
    //
    //    // RAII wrapper for GStreamer GstEvent* creation/destruction
    //    class Event
    //    {
    //    public:
    //        explicit Event(GstEvent* event) : m_event(event) {}
    //        ~Event() { gst_event_unref(m_event); }
    //
    //        operator GstEvent*() { return m_event; }
    //    private:
    //        GstEvent* m_event;
    //    };

    // RAII wrapper for GStreamer GError* creation/destruction
    class Error
    {
    public:
        explicit Error(GError* error) : m_error(error) {}
        ~Error() { g_clear_error(&m_error); }
        DECLARE_NOT_COPYABLE(Error);
        DECLARE_NOT_MOVABLE(Error);

        operator GError*() { return m_error; }

    private:
        GError* m_error;
    };

    std::shared_ptr<gst::Gst> make_gst(int* argc, char** argv[])
    {
        return std::make_shared<gst::Gst>(argc, argv);
    }

    gst::Element element_factory_make(std::string_view factoryName, std::string_view name)
    {
        return gst::Element(gst_element_factory_make(factoryName.data(), name.data()));
    }
    gst::Bus element_get_bus(gst::Element& element)
    {
        return gst::Bus(gst_element_get_bus(element));
    }
    std::pair<gst::Error, gst::gchar> message_parse_error(gst::Message& message)
    {
        std::unique_ptr<GError> error;
        std::unique_ptr<gchar> debug;
        gst_message_parse_error(message, gst::internal::out_ptr<GError*>(error), gst::internal::out_ptr(debug));
        return {gst::Error(error.get()), gst::gchar(debug.get())};
    }
}
