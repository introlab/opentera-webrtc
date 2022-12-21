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
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/mediastream/libwebrtc/GStreamerVideoDecoderFactory.cpp.auto.html
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/mediastream/gstreamer/GStreamerVideoFrameLibWebRTC.h.auto.html
 */

#include <OpenteraWebrtcNativeGStreamer/Decoders/GStreamerVideoDecoder.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GstMappedFrame.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/out_ptr.h>

#include <libyuv.h>

#ifdef DEBUG_GSTREAMER
#include <iostream>
#endif

using namespace opentera;
using namespace std;

constexpr size_t GstDecoderBufferSize = 10 * 1024 * 1024;
constexpr size_t WebrtcBufferPoolSize = 300;

GStreamerVideoDecoder::GStreamerVideoDecoder(string mediaTypeCaps, string decoderPipeline)
    : m_mediaTypeCaps{move(mediaTypeCaps)},
      m_decoderPipeline(move(decoderPipeline)),
      m_gstDecoderPipeline{},
      m_keyframeNeeded{true},
      m_firstBufferPts{GST_CLOCK_TIME_NONE},
      m_firstBufferDts{GST_CLOCK_TIME_NONE},
      m_width{0},
      m_height{0},
      m_imageReadyCb{nullptr},
      m_webrtcBufferPool{false, WebrtcBufferPoolSize}
{
}

int32_t GStreamerVideoDecoder::Release()
{
    if (m_gstDecoderPipeline && m_gstDecoderPipeline->pipeline())
    {
        auto bus = gst::unique_from_ptr(gst_pipeline_get_bus(GST_PIPELINE(m_gstDecoderPipeline->pipeline())));
        gst_bus_set_sync_handler(bus.get(), nullptr, nullptr, nullptr);

        m_gstDecoderPipeline.reset();
    }
    return WEBRTC_VIDEO_CODEC_OK;
}

int32_t GStreamerVideoDecoder::Decode(const webrtc::EncodedImage& inputImage, bool missingFrames, int64_t renderTimeMs)
{
    if (m_keyframeNeeded)
    {
        if (inputImage._frameType != webrtc::VideoFrameType::kVideoFrameKey)
        {
            GST_WARNING("Waiting for keyframe but got a delta unit... asking for keyframe");
            return WEBRTC_VIDEO_CODEC_ERROR;
        }
        else
        {
            m_keyframeNeeded = false;
        }
    }

    if (!m_gstDecoderPipeline)
    {
        GST_ERROR("No source set, can't decode.");
        return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
    }


    // No idea why this is needed, taken from WebKit implementation
    if (!GST_CLOCK_TIME_IS_VALID(m_firstBufferPts))
    {
        gst::unique_ptr<GstPad> srcpad =
            gst::unique_from_ptr(gst_element_get_static_pad(m_gstDecoderPipeline->src(), "src"));
        m_firstBufferPts = (static_cast<guint64>(renderTimeMs)) * GST_MSECOND;
        m_firstBufferDts = (static_cast<guint64>(inputImage.Timestamp())) * GST_MSECOND;
    }

    gst::unique_ptr<GstBuffer> buffer;
    switch (gst_buffer_pool_acquire_buffer(m_bufferPool.get(), out_ptr(buffer), nullptr))
    {
        case GST_FLOW_OK:
            break;
        case GST_FLOW_FLUSHING:
            return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
        default:
            return WEBRTC_VIDEO_CODEC_ERROR;
    }
    if (inputImage.size() > gst_buffer_get_size(buffer.get()))
    {
        GST_ERROR("The buffer is too small");
        return WEBRTC_VIDEO_CODEC_ERROR;
    }
    gst_buffer_fill(buffer.get(), 0, inputImage.data(), inputImage.size());
    gst_buffer_set_size(buffer.get(), inputImage.size());

    GST_BUFFER_DTS(buffer.get()) = (static_cast<guint64>(inputImage.Timestamp()) * GST_MSECOND) - m_firstBufferDts;
    GST_BUFFER_PTS(buffer.get()) = (static_cast<guint64>(renderTimeMs) * GST_MSECOND) - m_firstBufferPts;
    InputTimestamps timestamps = {inputImage.Timestamp(), renderTimeMs};
    m_dtsPtsMap[GST_BUFFER_PTS(buffer.get())] = timestamps;

    auto sample = gst::unique_from_ptr(gst_sample_new(buffer.get(), getCapsForFrame(inputImage), nullptr, nullptr));

#ifdef DEBUG_GSTREAMER
    GST_WARNING("Pushing sample: %" GST_PTR_FORMAT, sample.get());
    GST_WARNING("Width: %d, Height: %d, Size: %lu", m_width, m_height, gst_buffer_get_size(m_buffer.get()));
#endif

    switch (gst_app_src_push_sample(GST_APP_SRC(m_gstDecoderPipeline->src()), sample.get()))
    {
        case GST_FLOW_OK:
            break;
        case GST_FLOW_FLUSHING:
            return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
        default:
            return WEBRTC_VIDEO_CODEC_ERROR;
    }

#ifdef DEBUG_GSTREAMER
    GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(
        GST_BIN(m_gstAppPipeline->pipeline()),
        GST_DEBUG_GRAPH_SHOW_ALL,
        "pipeline-push-sample");
    cout << "Sample (push) is " << hex << sample.get() << dec << endl;
#endif

    return pullSample();
}

bool GStreamerVideoDecoder::Configure(const webrtc::VideoDecoder::Settings& settings)
{
    m_keyframeNeeded = true;

    m_gstDecoderPipeline = make_unique<GStreamerDecoderPipeline>();
    if (WEBRTC_VIDEO_CODEC_OK != m_gstDecoderPipeline->init(m_mediaTypeCaps, m_decoderPipeline))
    {
        return false;
    }

    if (settings.buffer_pool_size().has_value())
    {
        if (!m_webrtcBufferPool.Resize(*settings.buffer_pool_size()))
        {
            return false;
        }
    }

    m_bufferPool = gst::unique_from_ptr(gst_buffer_pool_new());
    GstStructure* bufferPoolConfig = gst_buffer_pool_get_config(m_bufferPool.get());
    gst_buffer_pool_config_set_params(bufferPoolConfig, nullptr, GstDecoderBufferSize, 1, 0);
    if (!gst_buffer_pool_set_config(m_bufferPool.get(), bufferPoolConfig) ||
        !gst_buffer_pool_set_active(m_bufferPool.get(), true))
    {
        return false;
    }

    return true;
}

int32_t GStreamerVideoDecoder::RegisterDecodeCompleteCallback(webrtc::DecodedImageCallback* callback)
{
    m_imageReadyCb = callback;
    return WEBRTC_VIDEO_CODEC_OK;
}

int32_t GStreamerVideoDecoder::pullSample()
{
    GstState state;
    GstState pending;
    gst_element_get_state(m_gstDecoderPipeline->sink(), &state, &pending, GST_SECOND / 100);

#ifdef DEBUG_GSTREAMER
    GST_ERROR("State: %s; Pending: %s", gst_element_state_get_name(state), gst_element_state_get_name(pending));

    GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(
        GST_BIN(m_gstAppPipeline->pipeline()),
        GST_DEBUG_GRAPH_SHOW_ALL,
        "pipeline-pull-sample");
#endif

    auto sample = gst::unique_from_ptr(
        gst_app_sink_try_pull_sample(GST_APP_SINK(m_gstDecoderPipeline->sink()), GST_SECOND / 100));

    if (!sample)
    {
        if (!m_gstDecoderPipeline->ready() || state != GST_STATE_PLAYING)
        {
            GST_ERROR("Not ready");
            return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
        }
        else
        {
            GST_ERROR("Needs more data");
            return WEBRTC_VIDEO_CODEC_OK;
        }
    }

    m_gstDecoderPipeline->setReady(true);

#ifdef DEBUG_GSTREAMER
    cout << "Sample (pull) is " << hex << sample.get() << dec << endl;
#endif

    auto buffer = gst_sample_get_buffer(sample.get());

    // Make sure that the frame.timestamp == previsouly input_frame._timeStamp
    // as it is required by the VideoDecoder baseclass.
    // Taken from the WebKit implementation
    auto timestamps = m_dtsPtsMap[GST_BUFFER_PTS(buffer)];
    m_dtsPtsMap.erase(GST_BUFFER_PTS(buffer));

#ifdef DEBUG_GSTREAMER
    GST_WARNING("Pulling sample: %" GST_PTR_FORMAT, sample.get());
    GST_WARNING("With size: %lu", gst_buffer_get_size(buffer));
    GstVideoInfo info;
    gst_video_info_from_caps(&info, gst_sample_get_caps(sample.get()));
    GST_VIDEO_INFO_FORMAT(&info);
#endif

    GstMappedFrame mappedFrame(sample.get(), GST_MAP_READ);
    if (!mappedFrame)
    {
        GST_ERROR("Could not map frame");
        return WEBRTC_VIDEO_CODEC_ERROR;
    }
    if (mappedFrame.format() != GST_VIDEO_FORMAT_I420)
    {
        GST_ERROR("Wrong format: It must be I420");
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    auto i420Buffer = m_webrtcBufferPool.CreateI420Buffer(mappedFrame.width(), mappedFrame.height());
    if (!i420Buffer.get())
    {
        GST_ERROR("Could not create an I420 buffer");
        return WEBRTC_VIDEO_CODEC_NO_OUTPUT;
    }

    libyuv::I420Copy(
        mappedFrame.componentData(0),
        mappedFrame.componentStride(0),
        mappedFrame.componentData(1),
        mappedFrame.componentStride(1),
        mappedFrame.componentData(2),
        mappedFrame.componentStride(2),
        i420Buffer->MutableDataY(),
        i420Buffer->StrideY(),
        i420Buffer->MutableDataU(),
        i420Buffer->StrideU(),
        i420Buffer->MutableDataV(),
        i420Buffer->StrideV(),
        mappedFrame.width(),
        mappedFrame.height());

#ifdef DEBUG_GSTREAMER
    GST_LOG_OBJECT(
        m_gstAppPipeline->pipeline(),
        "Output decoded frame! %d -> %" GST_PTR_FORMAT,
        frame.timestamp(),
        buffer);
#endif

    if (m_imageReadyCb)
    {
        webrtc::VideoFrame decodedImage = webrtc::VideoFrame::Builder()
                                              .set_video_frame_buffer(i420Buffer)
                                              .set_timestamp_rtp(static_cast<uint32_t>(timestamps.timestamp))
                                              .set_timestamp_ms(timestamps.renderTimeMs)
                                              .build();
        m_imageReadyCb->Decoded(decodedImage, absl::nullopt, absl::nullopt);
    }

    return WEBRTC_VIDEO_CODEC_OK;
}

GstCaps* GStreamerVideoDecoder::getCapsForFrame(const webrtc::EncodedImage& image)
{
    gint lastWidth = m_width;
    gint lastHeight = m_height;
    m_width = image._encodedWidth != 0 ? image._encodedWidth : m_width;
    m_height = image._encodedHeight != 0 ? image._encodedHeight : m_height;

    if (!m_caps || lastWidth != m_width || lastHeight != m_height)
    {
        m_caps = gst::unique_from_ptr(gst_caps_new_simple(
            m_mediaTypeCaps.c_str(),
            "width",
            G_TYPE_INT,
            m_width,
            "height",
            G_TYPE_INT,
            m_height,
            "alignment",
            G_TYPE_STRING,
            "au",
            nullptr));
    }

    return m_caps.get();
}
