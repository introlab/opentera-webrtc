#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerSupport.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerMessageHandling.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/out_ptr.h>

#include <gst/app/gstappsink.h>

#include <string>

using namespace opentera;
using namespace opentera::internal;
using namespace std;

bool gst::elementFactoryExists(const char* name)
{
    auto factory = gst::unique_from_ptr(gst_element_factory_find(name));
    return factory != nullptr;
}

bool gst::testEncoderDecoderPipeline(std::string_view encoderDecoderPipeline)
{
    constexpr GstClockTime Timeout = GST_SECOND / 10;

    string pipelineStr = "videotestsrc num-buffers=1 ! "
                         "capsfilter caps=video/x-raw,format=(string)I420,width=(int)64,height=(int)64 ! " +
                         string(encoderDecoderPipeline) +
                         " ! appsink name=sink";

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
