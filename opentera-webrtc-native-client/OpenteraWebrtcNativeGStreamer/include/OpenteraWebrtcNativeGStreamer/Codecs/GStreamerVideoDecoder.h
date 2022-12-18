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

#include <OpenteraWebrtcNativeGStreamer/Pipeline/GStreamerAppPipeline.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/ClassMacro.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <api/video_codecs/video_decoder.h>
#include <modules/video_coding/include/video_codec_interface.h>
#include <common_video/include/video_frame_buffer_pool.h>
#include <media/base/codec.h>

namespace opentera
{
    struct InputTimestamps
    {
        uint64_t timestamp;
        int64_t renderTimeMs;
    };

    class GStreamerVideoDecoder : public webrtc::VideoDecoder
    {
        // TODO: dependency injection for the pipeline to choose encoder/decoder and codec
        std::unique_ptr<GStreamerAppPipeline> m_gstAppPipeline; // TODO rename to gstAppPipeline

        // TODO: keyframe handling
        bool m_keyframeNeeded;
        GstClockTime m_firstBufferPts;
        GstClockTime m_firstBufferDts;
        std::map<GstClockTime, InputTimestamps> m_dtsPtsMap;
        gint m_width;
        gint m_height;
        gst::unique_ptr<GstCaps> m_caps;

        webrtc::DecodedImageCallback* m_imageReadyCb;

        gst::unique_ptr<GstBuffer> m_buffer;
        webrtc::VideoFrameBufferPool m_webrtcBufferPool;

    public:
        GStreamerVideoDecoder();
        ~GStreamerVideoDecoder() override = default;

        DECLARE_NOT_COPYABLE(GStreamerVideoDecoder);
        DECLARE_NOT_MOVABLE(GStreamerVideoDecoder);

        int32_t Release() final;

        int32_t Decode(const webrtc::EncodedImage& inputImage, bool missingFrames, int64_t renderTimeMs) final;

        // TODO: Should be attributes
        /** Uncomment matchings Caps() and Name() as needed based on the decoder used */
        //static const gchar* Caps() { return "video/x-h264"; }
        //static const gchar* Name() { return cricket::kH264CodecName; }
        static const gchar* Caps() { return "video/x-vp8"; }
        static const gchar* Name() { return cricket::kVp8CodecName; }
        //static const gchar* Caps() { return "video/x-vp9"; }
        //static const gchar* Name() { return cricket::kVp9CodecName; }

        webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        bool Configure(const webrtc::VideoDecoder::Settings& settings) override;

        int32_t RegisterDecodeCompleteCallback(webrtc::DecodedImageCallback* callback) final;

    private:
        int32_t pullSample();
        GstCaps* getCapsForFrame(const webrtc::EncodedImage& image);
    };
}

#endif
