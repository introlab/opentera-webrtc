/*
 *  Copyright (C) 2018 Metrological Group B.V.
 *  Copyright (C) 2018 Igalia S.L.
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
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/mediastream/gstreamer/GStreamerVideoFrameLibWebRTC.h.auto.html
 */

#pragma once

#include "utils/gstreamer_helpers.h"
#include <gst/gst.h>
#include <gst/video/video.h>
#include <memory>
#include <api/frame_transformer_interface.h>
// #include "OpenteraWebrtcNativeClient/Utils/ClassMacro.h"
#include <common_video/include/video_frame_buffer_pool.h>

namespace opentera
{
    gst::unique_ptr<GstSample> GStreamerSampleFromLibWebRTCVideoFrame(const webrtc::VideoFrame& frame);
    webrtc::VideoFrame LibWebRTCVideoFrameFromGStreamerSample(
        GstSample* sample,
        webrtc::VideoRotation videoRotation,
        int64_t timestamp,
        int64_t renderTimeMs);

    class GStreamerVideoFrameLibWebRTC : public rtc::RefCountedObject<webrtc::VideoFrameBuffer>
    {
    public:
        // TODO: directly take a gst::unique_ptr<GstSample> instead of a GstSample*
        GStreamerVideoFrameLibWebRTC(GstSample* sample, GstVideoInfo info)
            : m_sample{gst::shared_from_ptr(sample)},
              m_info{info},
              //   m_bufferPool{},
              m_i420{}
        {
        }

        ~GStreamerVideoFrameLibWebRTC() override = default;

        static rtc::scoped_refptr<webrtc::VideoFrameBuffer> create(GstSample*);

        GstSample* getSample();

        rtc::scoped_refptr<webrtc::I420BufferInterface> ToI420() final;

        const webrtc::I420BufferInterface* GetI420() const final;

        int width() const override { return GST_VIDEO_INFO_WIDTH(&m_info); }
        int height() const override { return GST_VIDEO_INFO_HEIGHT(&m_info); }

        auto& refCount() { return ref_count_; };

    private:
        webrtc::VideoFrameBuffer::Type type() const override;

        void createCachedI420() const;

        std::shared_ptr<GstSample> m_sample;
        GstVideoInfo m_info;
        // FIXME: Useful for reuse of buffers?
        // mutable webrtc::VideoFrameBufferPool m_bufferPool;
        mutable rtc::scoped_refptr<webrtc::I420Buffer> m_i420;
    };
}
