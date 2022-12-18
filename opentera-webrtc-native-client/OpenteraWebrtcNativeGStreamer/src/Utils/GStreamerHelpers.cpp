#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>

// RAII wrapper for GStreamer initialization/deinitialization
class GstInit
{
public:
    GstInit()
    {
        gst_init(nullptr, nullptr);
    }
    ~GstInit()
    {
        gst_deinit();
    }
};

GstInit gstInitInstance;
