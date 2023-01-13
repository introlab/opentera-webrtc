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

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_GSTREAMER_VIDEO_ENCODER_FACTORY_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_GSTREAMER_VIDEO_ENCODER_FACTORY_H

#include <OpenteraWebrtcNativeGStreamer/Pipeline/GStreamerEncoderPipeline.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerBufferPool.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/ClassMacro.h>

#include <api/video_codecs/video_encoder.h>
#include <modules/video_coding/include/video_codec_interface.h>
#include <common_video/include/video_frame_buffer_pool.h>
#include <media/base/codec.h>

#include <atomic>

namespace opentera
{
    class GStreamerVideoEncoder : public webrtc::VideoEncoder
    {
        std::string m_mediaTypeCaps;
        std::string m_encoderPipeline;
        std::string m_encoderBitRatePropertyName;
        BitRateUnit m_encoderBitRatePropertyUnit;
        std::string m_encoderKeyframeIntervalPropertyName;

        std::unique_ptr<GStreamerEncoderPipeline> m_gstEncoderPipeline;
        GStreamerBufferPool m_gstreamerBufferPool;

        gst::unique_ptr<GstVideoInfo> m_inputVideoInfo;
        gst::unique_ptr<GstCaps> m_inputVideoCaps;

        GstClockTime m_firstBufferPts;
        GstClockTime m_firstBufferDts;

        webrtc::EncodedImage m_encodedFrame;
        webrtc::EncodedImageCallback* m_imageReadyCb;

        bool m_dropNextFrame;
        std::atomic<absl::optional<uint32_t>> m_newBitRate;

    public:
        GStreamerVideoEncoder(
            std::string mediaTypeCaps,
            std::string encoderPipeline,
            std::string encoderBitRatePropertyName,
            BitRateUnit encoderBitRatePropertyUnit,
            std::string encoderKeyframeIntervalPropertyName);
        ~GStreamerVideoEncoder() override = default;

        DECLARE_NOT_COPYABLE(GStreamerVideoEncoder);
        DECLARE_NOT_MOVABLE(GStreamerVideoEncoder);

        int32_t Release() override;

        int InitEncode(const webrtc::VideoCodec* codecSettings, const VideoEncoder::Settings& settings) override;

        int32_t RegisterEncodeCompleteCallback(webrtc::EncodedImageCallback* callback) override;

        int32_t Encode(const webrtc::VideoFrame& frame, const std::vector<webrtc::VideoFrameType>* frameTypes) override;

        void SetRates(const RateControlParameters& parameters) override;

        webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

    protected:
        virtual int getKeyframeInterval(const webrtc::VideoCodec& codecSettings) = 0;

        virtual void populateCodecSpecificInfo(
            webrtc::CodecSpecificInfo& codecSpecificInfo,
            const webrtc::EncodedImage& encodedFrame) = 0;

    private:
        bool initializePipeline();
        void initializeBufferTimestamps(int64_t renderTimeMs, uint32_t imageTimestamp);

        gst::unique_ptr<GstSample> toGstSample(const webrtc::VideoFrame& frame);
        bool updateEncodedFrame(const webrtc::VideoFrame& frame, gst::unique_ptr<GstSample>& encodedSample);
        webrtc::VideoFrameType getWebrtcFrameType(gst::unique_ptr<GstSample>& encodedSample);
    };
}

#endif
