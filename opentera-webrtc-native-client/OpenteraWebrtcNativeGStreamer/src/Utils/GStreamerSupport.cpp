#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerSupport.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerMessageHandling.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/out_ptr.h>

#include <gst/app/gstappsink.h>

#include <string>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

using namespace opentera;
using namespace opentera::internal;
using namespace std;

bool gst::elementFactoryExists(const char* name)
{
    auto factory = gst::unique_from_ptr(gst_element_factory_find(name));
    return factory != nullptr;
}

static bool testEncoderDecoderPipeline(const string& encoderDecoderPipeline)
{
    constexpr GstClockTime Timeout = GST_SECOND;

    string pipelineStr = "videotestsrc num-buffers=1 ! "
                         "capsfilter caps=video/x-raw,format=(string)I420,width=(int)256,height=(int)256 ! " +
                         encoderDecoderPipeline + " ! appsink name=sink";

    gst::unique_ptr<GError> error;
    auto pipeline = gst::unique_from_ptr(GST_PIPELINE(gst_parse_launch(pipelineStr.c_str(), out_ptr(error))));
    if (error)
    {
        return false;
    }

    auto sink = gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(pipeline.get()), "sink"));
    connectBusMessageCallback(pipeline);

    if (gst_element_set_state(GST_ELEMENT(pipeline.get()), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
        return false;
    }

    GstState state;
    if (gst_element_get_state(GST_ELEMENT(pipeline.get()), &state, nullptr, Timeout) != GST_STATE_CHANGE_SUCCESS ||
        state != GST_STATE_PLAYING)
    {
        return false;
    }

    auto sample = gst::unique_from_ptr(gst_app_sink_try_pull_sample(GST_APP_SINK(sink.get()), Timeout));
    return sample != nullptr;
}

bool gst::testEncoderDecoderPipeline(const string& encoderDecoderPipeline)
{
    static unordered_map<string, bool> cache;
    static shared_mutex mutex;

    {
        shared_lock lock(mutex);
        auto it = cache.find(encoderDecoderPipeline);
        if (it != cache.end())
        {
            return it->second;
        }
    }

    {
        unique_lock lock(mutex);
        bool ok = ::testEncoderDecoderPipeline(encoderDecoderPipeline);
        cache[encoderDecoderPipeline] = ok;
        return ok;
    }
}
