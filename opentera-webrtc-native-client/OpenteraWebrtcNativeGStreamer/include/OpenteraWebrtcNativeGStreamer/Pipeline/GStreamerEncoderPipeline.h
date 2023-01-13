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
        bool m_resetPipelineOnPropertyChange;
        std::string m_capsStr;
        std::string m_encoderPipeline;

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
        int32_t setBitRate(uint32_t bitRate);
        int32_t setKeyframeInterval(int interval);

        GstFlowReturn pushSample(gst::unique_ptr<GstSample>& sample);
        gst::unique_ptr<GstSample> tryPullSample();

        int32_t initialize(
            std::string encoderBitRatePropertyName,
            BitRateUnit bitRatePropertyUnit,
            std::string keyframeIntervalPropertyName,
            bool resetPipelineOnPropertyChange,
            std::string capsStr,
            std::string encoderPipeline);

    private:
        template<class T>
        void setEncoderProperty(const std::string& name, T value);

        int32_t initializePipelineWithoutChangingState();
    };

    template<class T>
    void GStreamerEncoderPipeline::setEncoderProperty(const std::string& name, T value)
    {
        auto dotPosition = name.find('.');
        auto valueString = std::to_string(value);

        if (dotPosition == std::string::npos)
        {
            GST_INFO("Set encoder property - %s=%s", name.c_str(), valueString.c_str());
            g_object_set(m_encoder.get(), name.c_str(), value, nullptr);
        }
        else
        {
            // TODO test
            std::string parentName = name.substr(0, dotPosition);
            std::string childName = name.substr(dotPosition + 1);
            GST_INFO("Set encoder property - %s.%s=%s", parentName.c_str(), childName.c_str(), valueString.c_str());

            GstStructure* structure;
            g_object_get(m_encoder.get(), parentName.c_str(), &structure, nullptr);
            if (!structure)
            {
                structure = gst_structure_new_empty(parentName.c_str());
            }

            gst_structure_set(structure, childName.c_str(), value, nullptr);
            g_object_set(m_encoder.get(), parentName.c_str(), structure, nullptr);

            gchar* parentValues = gst_structure_to_string(structure);
            GST_INFO("Parent values - %s", parentValues);
            g_free(parentValues);
        }
    }
}

#endif
