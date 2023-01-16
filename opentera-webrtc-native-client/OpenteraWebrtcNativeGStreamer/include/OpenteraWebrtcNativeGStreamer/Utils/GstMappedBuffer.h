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
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/graphics/gstreamer/GStreamerCommon.h.auto.html
 */

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_GST_MAPPED_BUFFER_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_GST_MAPPED_BUFFER_H

#include <OpenteraWebrtcNativeGStreamer/Utils/ClassMacro.h>

#include <gst/gst.h>
#include <gst/video/video.h>

namespace opentera
{

    class GstMappedBuffer
    {
        GstBuffer* m_buffer;
        GstMapInfo m_bufferInfo;
        bool m_isValid = false;

    public:
        GstMappedBuffer(GstBuffer* buffer, GstMapFlags flags);
        ~GstMappedBuffer();

        DECLARE_NOT_COPYABLE(GstMappedBuffer);
        DECLARE_NOT_MOVABLE(GstMappedBuffer);

        [[nodiscard]] uint8_t* data() const;
        [[nodiscard]] size_t size() const;

        explicit operator bool() const { return m_isValid; }
    };

    inline GstMappedBuffer::GstMappedBuffer(GstBuffer* buffer, GstMapFlags flags) : m_buffer(buffer)
    {
        m_isValid = gst_buffer_map(buffer, &m_bufferInfo, flags);
    }

    inline GstMappedBuffer::~GstMappedBuffer()
    {
        if (m_isValid)
        {
            gst_buffer_unmap(m_buffer, &m_bufferInfo);
        }
        m_isValid = false;
    }

    inline uint8_t* GstMappedBuffer::data() const { return m_isValid ? m_bufferInfo.data : nullptr; }

    inline size_t GstMappedBuffer::size() const { return m_isValid ? m_bufferInfo.size : 0; }
}

#endif
