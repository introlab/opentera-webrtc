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
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <modules/video_coding/include/video_codec_interface.h>

using namespace opentera;
using namespace opentera::internal;
using namespace std;

GStreamerDecoderPipeline::GStreamerDecoderPipeline()
    : m_pipeline{nullptr},
      m_src{nullptr},
      m_sink{nullptr},
      m_ready{false}
{
}

GStreamerDecoderPipeline::~GStreamerDecoderPipeline()
{
    if (m_pipeline)
    {
        disconnectBusMessageCallback(m_pipeline);
    }
}

GstFlowReturn GStreamerDecoderPipeline::pushSample(gst::unique_ptr<GstSample>& sample)
{
    return gst_app_src_push_sample(GST_APP_SRC(m_src.get()), sample.get());
}

void GStreamerDecoderPipeline::getSinkState(GstState& state, GstState& pending)
{
    gst_element_get_state(m_sink.get(), &state, &pending, GST_SECOND / 10);
}

gst::unique_ptr<GstSample> GStreamerDecoderPipeline::tryPullSample()
{
    return gst::unique_from_ptr(gst_app_sink_try_pull_sample(GST_APP_SINK(m_sink.get()), GST_SECOND / 10));
}

int32_t GStreamerDecoderPipeline::initialize(string_view capsStr, string_view decoderPipeline)
{
    if (m_pipeline)
    {
        disconnectBusMessageCallback(m_pipeline);
    }

    string pipelineStr = string("appsrc name=src emit-signals=true is-live=true format=time caps=") + string(capsStr) +

                         " ! queue ! " + string(decoderPipeline) +

                         " ! capsfilter caps=video/x-raw,format=I420"

                         " ! queue"
                         " ! appsink name=sink sync=false";

    GST_INFO("Pipeline: %s", pipelineStr.c_str());
    m_pipeline = gst::unique_from_ptr(GST_PIPELINE(gst_parse_launch(pipelineStr.c_str(), out_ptr(m_error))));
    if (m_error)
    {
        GST_ERROR("Failed to create pipeline: %s", m_error->message);
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    m_src = gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(m_pipeline.get()), "src"));
    if (!m_src)
    {
        GST_ERROR_OBJECT(m_pipeline.get(), "The pipeline must contain an appsrc named src.");
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    m_sink = gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(m_pipeline.get()), "sink"));
    if (!m_sink)
    {
        GST_ERROR_OBJECT(m_pipeline.get(), "The pipeline must contain an appsink named sink.");
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    connectBusMessageCallback(m_pipeline);

    if (gst_element_set_state(GST_ELEMENT(m_pipeline.get()), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
        GST_ERROR_OBJECT(m_pipeline.get(), "Could not set state to PLAYING.");
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

#ifdef DEBUG_GSTREAMER
    GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline()), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");
#endif

    return WEBRTC_VIDEO_CODEC_OK;
}
