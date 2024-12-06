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

constexpr size_t GstDecoderBufferSize = 20 * 1024 * 1024;
constexpr size_t WebrtcBufferPoolSize = 300;

GStreamerVideoDecoder::GStreamerVideoDecoder(
    string mediaTypeCaps,
    string decoderPipeline,
    bool resetPipelineOnSizeChanges)
    : m_mediaTypeCaps{move(mediaTypeCaps)},
      m_decoderPipeline(move(decoderPipeline)),
      m_resetPipelineOnSizeChanges(resetPipelineOnSizeChanges),
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
    if (m_gstDecoderPipeline)
    {
        m_gstDecoderPipeline.reset();
    }
    return WEBRTC_VIDEO_CODEC_OK;
}

int32_t GStreamerVideoDecoder::Decode(
    const webrtc::EncodedImage& inputImage,
    [[maybe_unused]] bool missingFrames,
    int64_t renderTimeMs)
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
            initializeBufferTimestamps(renderTimeMs, inputImage.RtpTimestamp());
            m_keyframeNeeded = false;
        }
    }

    if (!m_gstDecoderPipeline)
    {
        GST_ERROR("No source set, can't decode.");
        return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
    }

    if (m_resetPipelineOnSizeChanges && inputImage._frameType == webrtc::VideoFrameType::kVideoFrameKey &&
        m_width != 0 && m_height != 0 &&
        (m_width != static_cast<gint>(inputImage._encodedWidth) ||
         m_height != static_cast<gint>(inputImage._encodedHeight)))
    {
        initializePipeline();
        initializeBufferTimestamps(renderTimeMs, inputImage.RtpTimestamp());
    }

    auto sample = toGstSample(inputImage, renderTimeMs);
    if (!sample)
    {
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

#ifdef DEBUG_GSTREAMER
    GST_WARNING("Pushing sample: %" GST_PTR_FORMAT, sample.get());
    GST_WARNING("Width: %d, Height: %d, Size: %lu", m_width, m_height, gst_buffer_get_size(m_buffer.get()));
#endif

    switch (m_gstDecoderPipeline->pushSample(sample))
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

    return pullSample(renderTimeMs, inputImage.RtpTimestamp(), inputImage.rotation_);
}

bool GStreamerVideoDecoder::Configure(const webrtc::VideoDecoder::Settings& settings)
{
    m_keyframeNeeded = true;

    if (!initializePipeline())
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

    return m_gstreamerBufferPool.initialize(GstDecoderBufferSize);
}

int32_t GStreamerVideoDecoder::RegisterDecodeCompleteCallback(webrtc::DecodedImageCallback* callback)
{
    m_imageReadyCb = callback;
    return WEBRTC_VIDEO_CODEC_OK;
}

bool GStreamerVideoDecoder::initializePipeline()
{
    m_gstDecoderPipeline = make_unique<GStreamerDecoderPipeline>();
    return m_gstDecoderPipeline->initialize(m_mediaTypeCaps, m_decoderPipeline) == WEBRTC_VIDEO_CODEC_OK;
}

void GStreamerVideoDecoder::initializeBufferTimestamps(int64_t renderTimeMs, uint32_t imageTimestamp)
{
    m_firstBufferPts = (static_cast<guint64>(renderTimeMs)) * GST_MSECOND;
    m_firstBufferDts = (static_cast<guint64>(imageTimestamp)) * GST_MSECOND;
}

gst::unique_ptr<GstSample>
    GStreamerVideoDecoder::toGstSample(const webrtc::EncodedImage& inputImage, int64_t renderTimeMs)
{
    gst::unique_ptr<GstBuffer> buffer = m_gstreamerBufferPool.acquireBuffer();
    if (!buffer)
    {
        return nullptr;
    }
    if (inputImage.size() > gst_buffer_get_size(buffer.get()))
    {
        GST_ERROR("The buffer is too small");
        return nullptr;
    }

    gst_buffer_fill(buffer.get(), 0, inputImage.data(), inputImage.size());
    gst_buffer_set_size(buffer.get(), static_cast<gssize>(inputImage.size()));

    GST_BUFFER_DTS(buffer.get()) = (static_cast<guint64>(inputImage.RtpTimestamp()) * GST_MSECOND) - m_firstBufferDts;
    GST_BUFFER_PTS(buffer.get()) = (static_cast<guint64>(renderTimeMs) * GST_MSECOND) - m_firstBufferPts;

    return gst::unique_from_ptr(gst_sample_new(buffer.get(), getCapsForFrame(inputImage), nullptr, nullptr));
}

int32_t GStreamerVideoDecoder::pullSample(int64_t renderTimeMs, uint32_t imageTimestamp, webrtc::VideoRotation rotation)
{
    GstState state;
    GstState pending;
    m_gstDecoderPipeline->getSinkState(state, pending);

#ifdef DEBUG_GSTREAMER
    GST_ERROR("State: %s; Pending: %s", gst_element_state_get_name(state), gst_element_state_get_name(pending));

    GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(
        GST_BIN(m_gstAppPipeline->pipeline()),
        GST_DEBUG_GRAPH_SHOW_ALL,
        "pipeline-pull-sample");
#endif

    auto sample = m_gstDecoderPipeline->tryPullSample();
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
    auto buffer = gst_sample_get_buffer(sample.get());

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
    if (!i420Buffer)
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
                                              .set_timestamp_rtp(imageTimestamp)
                                              .set_timestamp_ms(renderTimeMs)
                                              .set_rotation(rotation)
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
        if (m_mediaTypeCaps == "video/x-h264")
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
                "stream-format",
                G_TYPE_STRING,
                "byte-stream",
                nullptr));
        }
        else
        {
            m_caps = gst::unique_from_ptr(gst_caps_new_simple(
                m_mediaTypeCaps.c_str(),
                "width",
                G_TYPE_INT,
                m_width,
                "height",
                G_TYPE_INT,
                m_height,
                nullptr));
        }
    }

    return m_caps.get();
}
