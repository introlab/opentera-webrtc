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

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_PIPELINE_GSTREAMER_DECODER_PIPELINE_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_PIPELINE_GSTREAMER_DECODER_PIPELINE_H

#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>

#include <string_view>

namespace opentera
{
    class GStreamerDecoderPipeline
    {
        gst::unique_ptr<GstPipeline> m_pipeline;
        gst::unique_ptr<GstElement> m_src;
        gst::unique_ptr<GstElement> m_sink;
        gst::unique_ptr<GError> m_error;
        bool m_ready;

    public:
        GStreamerDecoderPipeline();
        ~GStreamerDecoderPipeline();

        GstFlowReturn pushSample(gst::unique_ptr<GstSample>& sample);
        void getSinkState(GstState& state, GstState& pending);
        gst::unique_ptr<GstSample> tryPullSample();

        [[nodiscard]] bool ready() const;
        void setReady(bool ready);

        int32_t initialize(std::string_view capsStr, std::string_view decoderPipeline);
    };

    inline bool GStreamerDecoderPipeline::ready() const { return m_ready; }

    inline void GStreamerDecoderPipeline::setReady(bool ready) { m_ready = ready; }
}

#endif
