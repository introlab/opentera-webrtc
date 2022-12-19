/*
 *  Copyright (C) 2022 IntRoLab
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <OpenteraWebrtcNativeGStreamer/Pipeline/GStreamerDecoderPipeline.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/out_ptr.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerMessageHandling.h>

#include <gst/gst.h>

#include <modules/video_coding/include/video_codec_interface.h>

using namespace opentera;
using namespace opentera::internal;
using namespace std;

GStreamerDecoderPipeline::GStreamerDecoderPipeline()
    : m_pipeline{nullptr},
      m_src{nullptr},
      m_sink{nullptr},
      m_width{0},
      m_height{0},
      m_ready{false}
{
}

GstElement* GStreamerDecoderPipeline::pipeline()
{
    return GST_ELEMENT(m_pipeline.get());
}
GstElement* GStreamerDecoderPipeline::src()
{
    return m_src.get();
}
GstElement* GStreamerDecoderPipeline::sink()
{
    return m_sink.get();
}

[[nodiscard]] bool GStreamerDecoderPipeline::ready() const
{
    return m_ready;
}
void GStreamerDecoderPipeline::setReady(bool ready)
{
    m_ready = ready;
}

int32_t GStreamerDecoderPipeline::init(string_view capsStr, string_view decoderPipeline)
{
    string pipelineStr = string("appsrc name=src emit-signals=true is-live=true format=time caps=") + string(capsStr) +

                         /** Uncomment to use a test source */
                         // R"( ! queue ! fakesink sync=false)"
                         // R"( videotestsrc is-live=true ! capsfilter caps=video/x-raw,width=640,height=480)"
                         // R"( ! videoconvert)"
                         // R"( ! vaapih264enc)"

                         " ! queue name=q2 ! " + string(decoderPipeline) +

                         " ! capsfilter caps=video/x-raw,format=(string)I420"

                         /** Uncomment to enable the gstreamer alternative display */
                         //" ! tee name=tee"

                         " ! queue"
                         " ! appsink name=sink sync=false";

    /** Uncomment to enable the gstreamer alternative display */
    //" tee. ! queue ! fpsdisplaysink sync=false";

    m_pipeline = gst::unique_from_ptr(GST_PIPELINE(gst_parse_launch(pipelineStr.c_str(), out_ptr(m_error))));

    if (m_error)
    {
        GST_ERROR("Failed to create pipeline: %s", m_error->message);
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    m_src = gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(pipeline()), "src"));
    m_sink = gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(pipeline()), "sink"));

    connectBusMessageCallback(m_pipeline);

    if (gst_element_set_state(pipeline(), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
        GST_ERROR_OBJECT(pipeline(), "Could not set state to PLAYING.");
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    /** Uncomment to generate the pipeline dot file */
    // GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline()), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");

    return WEBRTC_VIDEO_CODEC_OK;
}
