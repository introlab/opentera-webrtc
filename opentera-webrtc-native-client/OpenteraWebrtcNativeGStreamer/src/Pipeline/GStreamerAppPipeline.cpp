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

#include <OpenteraWebrtcNativeGStreamer/Pipeline/GStreamerAppPipeline.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/out_ptr.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerMessageHandling.h>

#include <gst/gst.h>

#include <modules/video_coding/include/video_codec_interface.h>

using namespace opentera;
using namespace opentera::internal;

GStreamerAppPipeline::GStreamerAppPipeline()
    : m_gst{nullptr, nullptr},
      m_pipeline{nullptr},
      m_appsrc{nullptr},
      m_appsink{nullptr},
      m_width{0},
      m_height{0},
      m_ready{false}
{
}

GstElement* GStreamerAppPipeline::pipeline()
{
    return GST_ELEMENT(m_pipeline.get());
}
GstElement* GStreamerAppPipeline::src()
{
    return m_appsrc.get();
}
GstElement* GStreamerAppPipeline::sink()
{
    return m_appsink.get();
}

[[nodiscard]] bool GStreamerAppPipeline::ready() const
{
    return m_ready;
}
void GStreamerAppPipeline::setReady(bool ready)
{
    m_ready = ready;
}

int32_t GStreamerAppPipeline::init(std::string_view caps_str)
{
    std::string pipelineStr = std::string(R"(appsrc name=src emit-signals=true is-live=true format=time caps=)") +
                              std::string(caps_str) +

                              /** Uncomment to use a test source */
                              // R"( ! queue ! fakesink sync=false)"
                              // R"( videotestsrc is-live=true ! capsfilter caps=video/x-raw,width=640,height=480)"
                              // R"( ! videoconvert)"
                              // R"( ! vaapih264enc)"

                              R"( ! queue name=q2)"

                              // TODO: Should be dynamic
                              /** Uncomment to choose decoder to use */
                              // R"( ! vaapivp9dec name=decode ! vaapipostproc)"
                              // R"( ! vaapivp8dec name=decode ! vaapipostproc)"
                              //R"( ! vaapih264dec name=decode ! vaapipostproc)"
                              // R"( ! vp8dec name=decode)"
                              // R"( ! vp9dec name=decode)"
                              R"( ! h264parse ! avdec_h264 name=decode)"

                              R"( ! capsfilter caps=video/x-raw,format=(string)I420)"

                              /** Comment to disable the gstreamer alternative display */
                              R"( ! tee name=tee)"

                              R"( ! queue)"
                              R"( ! appsink name=sink emit-signals=true sync=false)"

                              /** Comment to disable the gstreamer alternative display */
                              R"( tee. ! queue ! fpsdisplaysink sync=false)"

                              "";

    m_pipeline = gst::unique_from_ptr(GST_PIPELINE(gst_parse_launch(pipelineStr.c_str(), out_ptr(m_error))));

    if (m_error)
    {
        GST_ERROR("Failed to create pipeline: %s", m_error->message);
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    m_appsrc = gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(pipeline()), "src"));
    m_appsink = gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(pipeline()), "sink"));

    connectBusMessageCallback(m_pipeline);

    if (gst_element_set_state(pipeline(), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
        GST_ERROR_OBJECT(pipeline(), "Could not set state to PLAYING.");
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline()), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline"); // TODO remove or only debug?

    return WEBRTC_VIDEO_CODEC_OK;
}
