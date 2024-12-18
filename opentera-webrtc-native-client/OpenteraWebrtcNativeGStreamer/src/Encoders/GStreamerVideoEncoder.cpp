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
 *  *
 * https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/mediastream/gstreamer/GStreamerVideoFrameLibWebRTC.cpp.auto.html
 */

#include <OpenteraWebrtcNativeGStreamer/Encoders/GStreamerVideoEncoder.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GstMappedFrame.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GstMappedBuffer.h>

#include <modules/video_coding/utility/simulcast_utility.h>

#include <libyuv.h>

using namespace opentera;
using namespace std;

GStreamerVideoEncoder::GStreamerVideoEncoder(
    string mediaTypeCaps,
    string encoderPipeline,
    string encoderBitRatePropertyName,
    BitRateUnit encoderBitRatePropertyUnit,
    string encoderKeyframeIntervalPropertyName)
    : m_mediaTypeCaps(move(mediaTypeCaps)),
      m_encoderPipeline(move(encoderPipeline)),
      m_encoderBitRatePropertyName(move(encoderBitRatePropertyName)),
      m_encoderBitRatePropertyUnit(encoderBitRatePropertyUnit),
      m_encoderKeyframeIntervalPropertyName(move(encoderKeyframeIntervalPropertyName)),
      m_firstBufferPts{GST_CLOCK_TIME_NONE},
      m_firstBufferDts{GST_CLOCK_TIME_NONE},
      m_imageReadyCb{nullptr},
      m_dropNextFrame(false)
{
}

int32_t GStreamerVideoEncoder::Release()
{
    if (m_gstEncoderPipeline)
    {
        m_gstEncoderPipeline.reset();
    }
    return WEBRTC_VIDEO_CODEC_OK;
}

int GStreamerVideoEncoder::InitEncode(
    const webrtc::VideoCodec* codecSettings,
    [[maybe_unused]] const VideoEncoder::Settings& settings)
{
    if (codecSettings == nullptr || codecSettings->maxFramerate < 1 ||
        (codecSettings->maxBitrate > 0 && codecSettings->startBitrate > codecSettings->maxBitrate) ||
        codecSettings->width < 1 || codecSettings->height < 1)
    {
        return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
    }
    if (webrtc::SimulcastUtility::NumberOfSimulcastStreams(*codecSettings) > 1)
    {
        GST_ERROR("Simulcast not supported.");
        return WEBRTC_VIDEO_CODEC_ERR_SIMULCAST_PARAMETERS_NOT_SUPPORTED;
    }

    GST_INFO("Initializing encoder (%s)", GetEncoderInfo().implementation_name.c_str());
    if (!initializePipeline())
    {
        return WEBRTC_VIDEO_CODEC_ERROR;
    }
    m_gstEncoderPipeline->setBitRate(codecSettings->startBitrate * 1000);
    m_gstEncoderPipeline->setKeyframeInterval(getKeyframeInterval(*codecSettings));

    m_inputVideoInfo = gst::unique_from_ptr(gst_video_info_new());
    gst_video_info_set_format(
        m_inputVideoInfo.get(),
        GST_VIDEO_FORMAT_I420,
        codecSettings->width,
        codecSettings->height);

    m_inputVideoCaps = gst::unique_from_ptr(gst_video_info_to_caps(m_inputVideoInfo.get()));

    if (!m_gstreamerBufferPool.initialize(GST_VIDEO_INFO_SIZE(m_inputVideoInfo)))
    {
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    m_encodedFrame.SetEncodedData(webrtc::EncodedImageBuffer::Create(GST_VIDEO_INFO_SIZE(m_inputVideoInfo)));
    m_encodedFrame._encodedWidth = codecSettings->width;
    m_encodedFrame._encodedHeight = codecSettings->height;

    return WEBRTC_VIDEO_CODEC_OK;
}

int32_t GStreamerVideoEncoder::RegisterEncodeCompleteCallback(webrtc::EncodedImageCallback* callback)
{
    m_imageReadyCb = callback;
    return WEBRTC_VIDEO_CODEC_OK;
}

int32_t GStreamerVideoEncoder::Encode(const webrtc::VideoFrame& frame, const vector<webrtc::VideoFrameType>* frameTypes)
{
    if (!m_imageReadyCb)
    {
        GST_ERROR("The encoded image callback is not set.");
        return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
    }
    if (!m_gstEncoderPipeline)
    {
        GST_ERROR("The pipeline is not created.");
        return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
    }

    if (!GST_CLOCK_TIME_IS_VALID(m_firstBufferPts) || !GST_CLOCK_TIME_IS_VALID(m_firstBufferDts))
    {
        initializeBufferTimestamps(frame.render_time_ms(), frame.timestamp());
    }

    bool keyFrameNeeded = any_of(
        frameTypes->begin(),
        frameTypes->end(),
        [](const webrtc::VideoFrameType& t) { return t == webrtc::VideoFrameType::kVideoFrameKey; });
    if (keyFrameNeeded)
    {
        m_gstEncoderPipeline->forceKeyFrame();
    }
    if (m_dropNextFrame)
    {
        m_dropNextFrame = false;
        return WEBRTC_VIDEO_CODEC_OK;
    }
    auto newBitRate = m_newBitRate.exchange(absl::nullopt);
    if (newBitRate.has_value())
    {
        m_gstEncoderPipeline->setBitRate(*newBitRate);
    }

    auto sample = toGstSample(frame);
    if (!sample)
    {
        return WEBRTC_VIDEO_CODEC_ERROR;
    }
    m_gstEncoderPipeline->pushSample(sample);

    auto encodedSample = m_gstEncoderPipeline->tryPullSample();
    if (!encodedSample)
    {
        GST_ERROR("No encoded sample available");
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    if (!updateEncodedFrame(frame, encodedSample))
    {
        return WEBRTC_VIDEO_CODEC_ERROR;
    }

    webrtc::CodecSpecificInfo codecSpecificInfo;
    populateCodecSpecificInfo(codecSpecificInfo, m_encodedFrame);

    auto result = m_imageReadyCb->OnEncodedImage(m_encodedFrame, &codecSpecificInfo);
    if (result.error != webrtc::EncodedImageCallback::Result::OK)
    {
        return WEBRTC_VIDEO_CODEC_ERROR;
    }
    m_dropNextFrame = result.drop_next_frame;

    return WEBRTC_VIDEO_CODEC_OK;
}

void GStreamerVideoEncoder::SetRates(const RateControlParameters& parameters)
{
    m_newBitRate.store(parameters.bitrate.get_sum_bps());
}

webrtc::VideoEncoder::EncoderInfo GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info;
    info.requested_resolution_alignment = 2;
    info.supports_simulcast = false;
    info.preferred_pixel_formats = {webrtc::VideoFrameBuffer::Type::kI420};

    return info;
}

bool GStreamerVideoEncoder::initializePipeline()
{
    m_gstEncoderPipeline = make_unique<GStreamerEncoderPipeline>();
    return m_gstEncoderPipeline->initialize(
               m_encoderBitRatePropertyName,
               m_encoderBitRatePropertyUnit,
               m_encoderKeyframeIntervalPropertyName,
               m_mediaTypeCaps,
               m_encoderPipeline) == WEBRTC_VIDEO_CODEC_OK;
}

void GStreamerVideoEncoder::initializeBufferTimestamps(int64_t renderTimeMs, uint32_t imageTimestamp)
{
    m_firstBufferPts = (static_cast<guint64>(renderTimeMs)) * GST_MSECOND;
    m_firstBufferDts = (static_cast<guint64>(imageTimestamp)) * GST_MSECOND;
}

gst::unique_ptr<GstSample> GStreamerVideoEncoder::toGstSample(const webrtc::VideoFrame& frame)
{
    gst::unique_ptr<GstBuffer> buffer = m_gstreamerBufferPool.acquireBuffer();
    if (!buffer)
    {
        GST_ERROR("No buffer available");
        return nullptr;
    }

    auto i420Buffer = frame.video_frame_buffer()->ToI420();
    if (GST_VIDEO_INFO_WIDTH(m_inputVideoInfo.get()) != i420Buffer->width() ||
        GST_VIDEO_INFO_HEIGHT(m_inputVideoInfo.get()) != i420Buffer->height())
    {
        GST_ERROR("The input frame size is invalid");
        return nullptr;
    }

    GstMappedFrame mappedFrame(buffer.get(), m_inputVideoInfo.get(), GST_MAP_WRITE);
    if (!mappedFrame)
    {
        GST_ERROR("Could not map frame");
        return nullptr;
    }

    libyuv::I420Copy(
        i420Buffer->DataY(),
        i420Buffer->StrideY(),
        i420Buffer->DataU(),
        i420Buffer->StrideU(),
        i420Buffer->DataV(),
        i420Buffer->StrideV(),
        mappedFrame.componentData(0),
        mappedFrame.componentStride(0),
        mappedFrame.componentData(1),
        mappedFrame.componentStride(1),
        mappedFrame.componentData(2),
        mappedFrame.componentStride(2),
        i420Buffer->width(),
        i420Buffer->height());

    GST_BUFFER_DTS(buffer.get()) = (static_cast<guint64>(frame.timestamp()) * GST_MSECOND) - m_firstBufferDts;
    GST_BUFFER_PTS(buffer.get()) = (static_cast<guint64>(frame.render_time_ms()) * GST_MSECOND) - m_firstBufferPts;

    return gst::unique_from_ptr(gst_sample_new(buffer.get(), m_inputVideoCaps.get(), nullptr, nullptr));
}

bool GStreamerVideoEncoder::updateEncodedFrame(
    const webrtc::VideoFrame& frame,
    gst::unique_ptr<GstSample>& encodedSample)
{
    GstMappedBuffer mappedBuffer(gst_sample_get_buffer(encodedSample.get()), GST_MAP_READ);
    if (!mappedBuffer)
    {
        GST_ERROR("gst_buffer_map failed");
        return false;
    }

    if (m_encodedFrame.GetEncodedData()->size() < mappedBuffer.size())
    {
        m_encodedFrame.SetEncodedData(webrtc::EncodedImageBuffer::Create(2 * m_encodedFrame.GetEncodedData()->size()));
    }

    memcpy(m_encodedFrame.GetEncodedData()->data(), mappedBuffer.data(), mappedBuffer.size());

    m_encodedFrame.set_size(mappedBuffer.size());
    m_encodedFrame._frameType = getWebrtcFrameType(encodedSample);
    m_encodedFrame.capture_time_ms_ = frame.render_time_ms();
    m_encodedFrame.SetRtpTimestamp(frame.timestamp());
    m_encodedFrame.ntp_time_ms_ = frame.ntp_time_ms();
    m_encodedFrame.rotation_ = frame.rotation();

    return true;
}

webrtc::VideoFrameType GStreamerVideoEncoder::getWebrtcFrameType(gst::unique_ptr<GstSample>& encodedSample)
{
    if (GST_BUFFER_FLAG_IS_SET(gst_sample_get_buffer(encodedSample.get()), GST_BUFFER_FLAG_DELTA_UNIT))
    {
        return webrtc::VideoFrameType::kVideoFrameDelta;
    }
    else
    {
        return webrtc::VideoFrameType::kVideoFrameKey;
    }
}
