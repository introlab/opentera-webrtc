#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerSupport.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>

bool gst::elementFactoryExists(const char* name)
{
    auto factory = gst::unique_from_ptr(gst_element_factory_find(name));
    return factory != nullptr;
}
