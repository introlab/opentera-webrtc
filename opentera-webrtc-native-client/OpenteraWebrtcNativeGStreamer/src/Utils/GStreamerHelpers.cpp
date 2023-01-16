#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>

// RAII wrapper for GStreamer initialization/deinitialization
class GstInit
{
public:
    GstInit()
    {
        if (!gst_is_initialized())
        {
            gst_init(nullptr, nullptr);
        }
    }
};

GstInit gstInitInstance;
