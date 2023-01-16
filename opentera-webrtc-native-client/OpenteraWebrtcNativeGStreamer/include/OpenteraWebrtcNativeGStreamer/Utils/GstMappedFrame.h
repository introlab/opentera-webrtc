/*
 *  Copyright (C) 2012, 2015, 2016, 2018 Igalia S.L
 *  Copyright (C) 2015, 2016, 2018 Metrological Group B.V.
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
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/mediastream/gstreamer/GStreamerVideoFrameLibWebRTC.cpp.auto.html
 */

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_GST_MAPPED_FRAME_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_GST_MAPPED_FRAME_H

#include <OpenteraWebrtcNativeGStreamer/Utils/ClassMacro.h>

#include <gst/gst.h>
#include <gst/video/video.h>

namespace opentera
{

    class GstMappedFrame
    {
        GstVideoFrame m_frame;
        bool m_isValid = false;

    public:
        GstMappedFrame(GstSample* sample, GstMapFlags flags);
        GstMappedFrame(GstBuffer* buffer, GstVideoInfo* info, GstMapFlags flags);
        ~GstMappedFrame();

        DECLARE_NOT_COPYABLE(GstMappedFrame);
        DECLARE_NOT_MOVABLE(GstMappedFrame);

        uint8_t* componentData(int comp);
        int componentStride(int stride);

        int width();
        int height();
        int format();

        explicit operator bool() const { return m_isValid; }
    };

    inline GstMappedFrame::GstMappedFrame(GstSample* sample, GstMapFlags flags)
    {
        GstVideoInfo info;

        if (!gst_video_info_from_caps(&info, gst_sample_get_caps(sample)))
        {
            m_isValid = false;
            m_frame = {};
        }
        else
        {
            m_isValid = gst_video_frame_map(&m_frame, &info, gst_sample_get_buffer(sample), flags);
        }
    }

    inline GstMappedFrame::GstMappedFrame(GstBuffer* buffer, GstVideoInfo* info, GstMapFlags flags)
    {
        m_isValid = gst_video_frame_map(&m_frame, info, buffer, flags);
    }

    inline GstMappedFrame::~GstMappedFrame()
    {
        if (m_isValid)
        {
            gst_video_frame_unmap(&m_frame);
        }
        m_isValid = false;
    }

    inline uint8_t* GstMappedFrame::componentData(int comp)
    {
        return m_isValid ? GST_VIDEO_FRAME_COMP_DATA(&m_frame, comp) : nullptr;
    }

    inline int GstMappedFrame::componentStride(int stride)
    {
        return m_isValid ? GST_VIDEO_FRAME_COMP_STRIDE(&m_frame, stride) : -1;
    }

    inline int GstMappedFrame::width() { return m_isValid ? GST_VIDEO_FRAME_WIDTH(&m_frame) : -1; }

    inline int GstMappedFrame::height() { return m_isValid ? GST_VIDEO_FRAME_HEIGHT(&m_frame) : -1; }

    inline int GstMappedFrame::format()
    {
        return m_isValid ? GST_VIDEO_FRAME_FORMAT(&m_frame) : GST_VIDEO_FORMAT_UNKNOWN;
    }
}

#endif
