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

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_PIPELINE_GSTREAMER_APP_PIPELINE_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_PIPELINE_GSTREAMER_APP_PIPELINE_H

#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>

#include <string_view>

namespace opentera
{
    // TODO: Generalize this class to be able to use it for both encoding and decoding, and any encoder/encoder
    class GStreamerAppPipeline
    {
    public:
        GStreamerAppPipeline();

        GstElement* pipeline();
        GstElement* src();
        GstElement* sink();

        [[nodiscard]] bool ready() const;
        void setReady(bool ready);

        int32_t init(std::string_view caps_str);

    protected:


    private:
        // TODO: Share gst::Gst with all encoders/decoders to keep gstreamer alive as long as any of them is alive
        gst::Gst m_gst;
        gst::unique_ptr<GstPipeline> m_pipeline;
        gst::unique_ptr<GstElement> m_appsrc;
        gst::unique_ptr<GstElement> m_appsink;
        gst::unique_ptr<GError> m_error;
        gint m_width;
        gint m_height;
        bool m_ready;
    };
}

#endif
