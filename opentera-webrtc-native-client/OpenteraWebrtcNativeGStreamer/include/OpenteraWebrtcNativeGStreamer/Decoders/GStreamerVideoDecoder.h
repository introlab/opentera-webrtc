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

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_GSTREAMER_VIDEO_DECODER_FACTORY_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_GSTREAMER_VIDEO_DECODER_FACTORY_H

#include <OpenteraWebrtcNativeGStreamer/Pipeline/GStreamerDecoderPipeline.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerBufferPool.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/ClassMacro.h>

#include <api/video_codecs/video_decoder.h>
#include <modules/video_coding/include/video_codec_interface.h>
#include <common_video/include/video_frame_buffer_pool.h>
#include <media/base/codec.h>

namespace opentera
{
    class GStreamerVideoDecoder : public webrtc::VideoDecoder
    {
        std::string m_mediaTypeCaps;
        std::string m_decoderPipeline;
        bool m_resetPipelineOnSizeChanges;
        std::unique_ptr<GStreamerDecoderPipeline> m_gstDecoderPipeline;
        GStreamerBufferPool m_gstreamerBufferPool;

        bool m_keyframeNeeded;
        GstClockTime m_firstBufferPts;
        GstClockTime m_firstBufferDts;
        gint m_width;
        gint m_height;
        gst::unique_ptr<GstCaps> m_caps;

        webrtc::DecodedImageCallback* m_imageReadyCb;
        webrtc::VideoFrameBufferPool m_webrtcBufferPool;

    public:
        GStreamerVideoDecoder(
            std::string mediaTypeCaps,
            std::string decoderPipeline,
            bool resetPipelineOnSizeChanges = false);
        ~GStreamerVideoDecoder() override = default;

        DECLARE_NOT_COPYABLE(GStreamerVideoDecoder);
        DECLARE_NOT_MOVABLE(GStreamerVideoDecoder);

        int32_t Release() override;

        int32_t Decode(const webrtc::EncodedImage& inputImage, bool missingFrames, int64_t renderTimeMs) override;

        bool Configure(const webrtc::VideoDecoder::Settings& settings) override;

        int32_t RegisterDecodeCompleteCallback(webrtc::DecodedImageCallback* callback) override;

    private:
        bool initializePipeline();
        void initializeBufferTimestamps(int64_t renderTimeMs, uint32_t imageTimestamp);

        gst::unique_ptr<GstSample> toGstSample(const webrtc::EncodedImage& inputImage, int64_t renderTimeMs);

        int32_t pullSample(int64_t renderTimeMs, uint32_t imageTimestamp, webrtc::VideoRotation rotation);
        GstCaps* getCapsForFrame(const webrtc::EncodedImage& image);
    };
}

#endif
