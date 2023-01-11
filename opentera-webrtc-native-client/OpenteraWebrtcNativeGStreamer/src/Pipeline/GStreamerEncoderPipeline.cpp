/*
 *  Copyright (C) 2018, 2020 Metrological Group B.V.
 *  Copyright (C) 2018, 2020 Igalia S.L.
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

/*
 *  Original file(s):
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/mediastream/libwebrtc/GStreamerVideoEncoderFactory.cpp.auto.html
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/mediastream/gstreamer/GStreamerVideoFrameLibWebRTC.h.auto.html
 */

#include <OpenteraWebrtcNativeGStreamer/Pipeline/GStreamerEncoderPipeline.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/out_ptr.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerMessageHandling.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <modules/video_coding/include/video_codec_interface.h>

#include <cmath>

using namespace opentera;
using namespace opentera::internal;
using namespace std;

GStreamerEncoderPipeline::GStreamerEncoderPipeline()
    : m_bitRatePropertyUnit(BitRateUnit::BitPerSec),
      m_setPipelineStateToReadyOnPropertyChange(false)
{
}

GStreamerEncoderPipeline::~GStreamerEncoderPipeline()
{
    if (m_pipeline)
    {
        disconnectBusMessageCallback(m_pipeline);
    }
}

void GStreamerEncoderPipeline::forceKeyFrame()
{
    gst_pad_push_event(
        m_srcPad.get(),
        gst_video_event_new_downstream_force_key_unit(
            GST_CLOCK_TIME_NONE,
            GST_CLOCK_TIME_NONE,
            GST_CLOCK_TIME_NONE,
            FALSE,
            1));
}

void GStreamerEncoderPipeline::setBitRate(uint32_t bitRate)
{
    // TODO check property name and unit
    if (!m_encoder)
    {
        return;
    }

    if (m_setPipelineStateToReadyOnPropertyChange)
    {
        gst_element_set_state(GST_ELEMENT(m_pipeline.get()), GST_STATE_READY);
    }

    gint scaledBitRate = 0;
    switch (m_bitRatePropertyUnit)
    {
        case BitRateUnit::BitPerSec:
            scaledBitRate = static_cast<gint>(bitRate);
            break;
        case BitRateUnit::KBitPerSec:
            scaledBitRate = static_cast<gint>(round(static_cast<float>(bitRate) / 1000.f));
            break;
    }

    setEncoderProperty(m_encoderBitratePropertyName, scaledBitRate);

    if (m_setPipelineStateToReadyOnPropertyChange)
    {
        gst_element_set_state(GST_ELEMENT(m_pipeline.get()), GST_STATE_PLAYING);
    }
}

void GStreamerEncoderPipeline::setKeyframeInterval(int interval)
{
    // TODO check property name
    if (!m_encoder)
    {
        return;
    }
    if (m_setPipelineStateToReadyOnPropertyChange)
    {
        gst_element_set_state(GST_ELEMENT(m_pipeline.get()), GST_STATE_READY);
    }

    setEncoderProperty(m_encoderBitratePropertyName, static_cast<guint>(interval));

    if (m_setPipelineStateToReadyOnPropertyChange)
    {
        gst_element_set_state(GST_ELEMENT(m_pipeline.get()), GST_STATE_PLAYING);
    }
}

GstFlowReturn GStreamerEncoderPipeline::pushSample(gst::unique_ptr<GstSample>& sample)
{
    return gst_app_src_push_sample(GST_APP_SRC(m_src.get()), sample.get());
}

gst::unique_ptr<GstSample> GStreamerEncoderPipeline::tryPullSample()
{
    return gst::unique_from_ptr(gst_app_sink_try_pull_sample(GST_APP_SINK(m_sink.get()), GST_SECOND / 10));
}

int32_t GStreamerEncoderPipeline::initialize(
    string encoderBitratePropertyName,
    BitRateUnit bitRatePropertyUnit,
    string keyframeIntervalPropertyName,
    bool setPipelineStateToReadyOnPropertyChange,
    string_view capsStr,
    string_view encoderPipeline)
{
    m_encoderBitratePropertyName = move(encoderBitratePropertyName);
    m_bitRatePropertyUnit = bitRatePropertyUnit;
    m_keyframeIntervalPropertyName = move(keyframeIntervalPropertyName);
    m_setPipelineStateToReadyOnPropertyChange = setPipelineStateToReadyOnPropertyChange;

    if (m_pipeline)
    {
        disconnectBusMessageCallback(m_pipeline);
    }

    string pipelineStr =
        string("appsrc name=src emit-signals=true is-live=true format=time caps=video/x-raw,format=I420") +

        " ! queue ! " + string(encoderPipeline) +

        " ! capsfilter caps=" + string(capsStr) +

        " ! queue"
        " ! appsink name=sink sync=false";

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
    m_srcPad = gst::unique_from_ptr(gst_element_get_static_pad(m_src.get(), "src"));
    if (!m_srcPad)
    {
        GST_ERROR_OBJECT(m_pipeline.get(), "Unable to get the src pad of the appsrc");
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    m_encoder = gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(m_pipeline.get()), "encoder"));
    if (!m_encoder)
    {
        GST_ERROR_OBJECT(m_pipeline.get(), "The pipeline must contain an encoder named encoder.");
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
