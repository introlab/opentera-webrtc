/*
 *  Copyright (C) 2012, 2015, 2016 Igalia S.L
 *  Copyright (C) 2015, 2016 Metrological Group B.V.
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
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 *  Original file(s):
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/graphics/gstreamer/GStreamerCommon.cpp.auto.html
 */

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_GSTREAMER_MESSAGE_HANDLING_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_GSTREAMER_MESSAGE_HANDLING_H

#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>

namespace opentera::internal
{
    inline void busMessageCallback(GstBus*, GstMessage* message, GstBin* pipeline)
    {
        switch (GST_MESSAGE_TYPE(message))
        {
            case GST_MESSAGE_ERROR:
                GST_ERROR_OBJECT(pipeline, "Got message: %" GST_PTR_FORMAT, static_cast<void*>(message));
                {
#ifdef DEBUG_GSTREAMER
                    std::string dotFileName = std::string(GST_OBJECT_NAME(pipeline)) + "_error";
                    GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(pipeline, GST_DEBUG_GRAPH_SHOW_ALL, dotFileName.c_str());
#endif
                }
                break;
            case GST_MESSAGE_STATE_CHANGED:
                if (GST_MESSAGE_SRC(message) == GST_OBJECT(pipeline))
                {
                    GstState oldState, newState, pending;
                    gst_message_parse_state_changed(message, &oldState, &newState, &pending);

                    GST_INFO_OBJECT(
                        pipeline,
                        "State changed (old: %s, new: %s, pending: %s)",
                        gst_element_state_get_name(oldState),
                        gst_element_state_get_name(newState),
                        gst_element_state_get_name(pending));

#ifdef DEBUG_GSTREAMER
                    std::string dotFileName = std::string(GST_OBJECT_NAME(pipeline)) + "_" +
                                              gst_element_state_get_name(oldState) + "_" +
                                              gst_element_state_get_name(newState);
                    GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL, dotFileName.c_str());
#endif
                }
                break;
            default:
                break;
        }
    }

    inline void connectBusMessageCallback(gst::unique_ptr<GstPipeline>& pipeline)
    {
        auto bus = gst::unique_from_ptr(gst_pipeline_get_bus(GST_PIPELINE(pipeline.get())));
        gst_bus_add_signal_watch_full(bus.get(), 100);
        g_signal_connect(bus.get(), "message", G_CALLBACK(busMessageCallback), pipeline.get());
    }

    inline void disconnectBusMessageCallback(gst::unique_ptr<GstPipeline>& pipeline)
    {
        auto bus = gst::unique_from_ptr(gst_pipeline_get_bus(GST_PIPELINE(pipeline.get())));
        g_signal_handlers_disconnect_by_func(bus.get(), reinterpret_cast<gpointer>(busMessageCallback), pipeline.get());
    }
}  // namespace opentera::internal

#endif
