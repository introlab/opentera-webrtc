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

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_PIPELINE_GSTREAMER_ENCODER_PIPELINE_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_PIPELINE_GSTREAMER_ENCODER_PIPELINE_H

#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/out_ptr.h>

#include <string_view>

namespace opentera
{
    enum class BitRateUnit
    {
        BitPerSec,
        KBitPerSec
    };

    class GStreamerEncoderPipeline
    {
        std::string m_encoderBitRatePropertyName;
        BitRateUnit m_encoderBitRatePropertyUnit;
        std::string m_encoderKeyframeIntervalPropertyName;

        gst::unique_ptr<GstPipeline> m_pipeline;
        gst::unique_ptr<GstElement> m_src;
        gst::unique_ptr<GstPad> m_srcPad;
        gst::unique_ptr<GstElement> m_encoder;
        gst::unique_ptr<GstElement> m_sink;
        gst::unique_ptr<GError> m_error;

    public:
        GStreamerEncoderPipeline();
        ~GStreamerEncoderPipeline();

        void forceKeyFrame();
        void setBitRate(uint32_t bitRate);
        void setKeyframeInterval(int interval);

        GstFlowReturn pushSample(gst::unique_ptr<GstSample>& sample);
        gst::unique_ptr<GstSample> tryPullSample();

        int32_t initialize(
            std::string encoderBitRatePropertyName,
            BitRateUnit bitRatePropertyUnit,
            std::string keyframeIntervalPropertyName,
            std::string_view capsStr,
            std::string_view encoderPipeline);

    private:
        void setEncoderProperty(const std::string& name, guint value);
    };
}

#endif
