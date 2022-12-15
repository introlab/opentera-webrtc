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

#include "GStreamerFramesToFromWebrtc.h"

#include "OpenteraWebrtcNativeClient/Utils/ClassMacro.h"

// TODO: Split this file

namespace opentera
{
    class GstMappedFrame
    {
    public:
        GstMappedFrame(GstBuffer* buffer, GstVideoInfo info, GstMapFlags flags)
        {
            m_isValid = gst_video_frame_map(&m_frame, &info, buffer, flags);
        }

        // TODO: Choose if we keep these or define move/copy operations
        DECLARE_NOT_COPYABLE(GstMappedFrame);
        DECLARE_NOT_MOVABLE(GstMappedFrame);


        GstMappedFrame(GstSample* sample, GstMapFlags flags)
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

        GstVideoFrame* get()
        {
            if (!m_isValid)
            {
                GST_INFO("Invalid frame, returning NULL");
                return nullptr;
            }

            return &m_frame;
        }

        // TODO: Check if isValid
        uint8_t* ComponentData(int comp) { return GST_VIDEO_FRAME_COMP_DATA(&m_frame, comp); }
        int ComponentStride(int stride) { return GST_VIDEO_FRAME_COMP_STRIDE(&m_frame, stride); }

        GstVideoInfo* info()
        {
            if (!m_isValid)
            {
                GST_INFO("Invalid frame, returning NULL");
                return nullptr;
            }

            return &m_frame.info;
        }

        int width() { return m_isValid ? GST_VIDEO_FRAME_WIDTH(&m_frame) : -1; }
        int height() { return m_isValid ? GST_VIDEO_FRAME_HEIGHT(&m_frame) : -1; }
        int format() { return m_isValid ? GST_VIDEO_FRAME_FORMAT(&m_frame) : GST_VIDEO_FORMAT_UNKNOWN; }

        ~GstMappedFrame()
        {
            if (m_isValid)
            {
                gst_video_frame_unmap(&m_frame);
            }
            m_isValid = false;
        }

        explicit operator bool() const { return m_isValid; }

    private:
        GstVideoFrame m_frame;
        bool m_isValid = false;
    };

    // TODO: This is not reviewed, it will (probably?) be needed only for the encoder
    gst::unique_ptr<GstSample> GStreamerSampleFromLibWebRTCVideoFrame(const webrtc::VideoFrame& frame)
    {
        throw std::runtime_error("Not implemented");
        //        if (frame.video_frame_buffer()->type() == webrtc::VideoFrameBuffer::Type::kNative)
        //        {
        //            auto framebuffer = dynamic_cast<GStreamerVideoFrameLibWebRTC*>(frame.video_frame_buffer().get());
        //            auto gstsample = framebuffer->getSample();
        //
        //            GST_LOG("Reusing native GStreamer sample: %p", gstsample);
        //
        //            return gstsample;
        //        }

        // auto webrtcbuffer = frame.video_frame_buffer()->ToI420().get();
        // // FIXME - Check lifetime of those buffers.
        // const uint8_t* comps[3] = {webrtcbuffer->DataY(), webrtcbuffer->DataU(), webrtcbuffer->DataV()};

        // GstVideoInfo info;
        // gst_video_info_set_format(&info, GST_VIDEO_FORMAT_I420, frame.width(), frame.height());
        // auto buffer = gst::unique_from_ptr(gst_buffer_new());
        // for (gint i = 0; i < 3; i++)
        // {
        //     gsize compsize = GST_VIDEO_INFO_COMP_STRIDE(&info, i) * GST_VIDEO_INFO_COMP_HEIGHT(&info, i);

        //     GstMemory* comp = gst_memory_new_wrapped(
        //         GST_MEMORY_FLAG_PHYSICALLY_CONTIGUOUS | GST_MEMORY_FLAG_READONLY,
        //         const_cast<gpointer>(reinterpret_cast<const void*>(comps[i])),
        //         compsize,
        //         0,
        //         compsize,
        //         webrtcbuffer,
        //         nullptr);
        //     gst_buffer_append_memory(buffer.get(), comp);
        // }

        // auto caps = gst::unique_from_ptr(gst_video_info_to_caps(&info));
        // auto sample = gst::unique_from_ptr(gst_sample_new(buffer.get(), caps.get(), nullptr, nullptr));
        // return sample;
    }

    webrtc::VideoFrame LibWebRTCVideoFrameFromGStreamerSample(
        GstSample* sample,
        webrtc::VideoRotation rotation,
        int64_t timestamp,
        int64_t renderTimeMs)
    {
        auto frameBuffer = GStreamerVideoFrameLibWebRTC::create(sample);

        /** Uncomment to help debugging */
        // std::cout << "Sample (frameBuffer) is " << std::hex << sample << std::dec << std::endl;
        // std::cout << "Sample (from frameBuffer) is " << std::hex << frameBuffer.get() << std::dec << std::endl;

        webrtc::VideoFrame::Builder builder{};

        builder.set_video_frame_buffer(frameBuffer)
            .set_timestamp_rtp(timestamp)
            .set_timestamp_ms(renderTimeMs)
            .set_rotation(rotation);

        return builder.build();
    }

    rtc::scoped_refptr<webrtc::VideoFrameBuffer> GStreamerVideoFrameLibWebRTC::create(GstSample* sample)
    {
        GstVideoInfo info;

        if (!gst_video_info_from_caps(&info, gst_sample_get_caps(sample)))
        {
            throw std::runtime_error("Never reached");
        }
        return rtc::scoped_refptr<webrtc::VideoFrameBuffer>(new GStreamerVideoFrameLibWebRTC(sample, info));
    }

    webrtc::VideoFrameBuffer::Type GStreamerVideoFrameLibWebRTC::type() const
    {
        return Type::kNative;
    }

    GstSample* GStreamerVideoFrameLibWebRTC::getSample()
    {
        return m_sample.get();
    }

    const webrtc::I420BufferInterface* GStreamerVideoFrameLibWebRTC::GetI420() const
    {
        createCachedI420();
        if (m_i420)
        {
            return m_i420.get();
        }
        return nullptr;
    }


    rtc::scoped_refptr<webrtc::I420BufferInterface> GStreamerVideoFrameLibWebRTC::ToI420()
    {
        createCachedI420();
        return m_i420;
    }

    void GStreamerVideoFrameLibWebRTC::createCachedI420() const
    {
        if (m_i420)
        {
            return;
        }

        GstMappedFrame inFrame(m_sample.get(), GST_MAP_READ);

        if (!inFrame)
        {
            GST_WARNING("Could not map frame");
            throw std::runtime_error("Could not map frame");
        }

        /** Uncomment to help debugging */
        // std::cout << "Finished converting sample (" << std::hex << m_sample.get() << std::dec << ") to I420"
        //   << std::endl;

        /** Uncomment to help debugging */
        // std::cout << "F: " << (inFrame.format()) << "\n";
        // std::cout << "Y: " << (inFrame.ComponentStride(0)) << "\n";
        // std::cout << "U: " << (inFrame.ComponentStride(1)) << "\n";
        // std::cout << "V: " << (inFrame.ComponentStride(2)) << "\n";
        // std::cout << "W: " << (inFrame.width()) << "\n";
        // std::cout << "H: " << (inFrame.height()) << "\n";

        /** Uncomment to save the frame in a file */
        /* Then, convert it to an image using `ffmpeg -y -s 640x480 -pix_fmt yuv420p -i gstreamer.yuv gstreamer.jpg` */
        // FILE* f = fopen("gstreamer.yuv", "wb");
        // fwrite(
        //     inFrame.ComponentData(0),
        //     1,
        //     inFrame.ComponentStride(0) * inFrame.height(),
        //     f);  // Write Y color channel.
        // fwrite(
        //     inFrame.ComponentData(1),
        //     1,
        //     inFrame.ComponentStride(1) * inFrame.height() / 2,
        //     f);  // Write Y color channel.
        // fwrite(
        //     inFrame.ComponentData(2),
        //     1,
        //     inFrame.ComponentStride(2) * inFrame.height() / 2,
        //     f);  // Write Y color channel.
        // fclose(f);

        if (inFrame.format() != GST_VIDEO_FORMAT_I420)
        {
            throw std::runtime_error("Unsupported format, make sure that the end of the GStreamer pipeline is I420");
        }

        // TODO: Reuse the buffer to reduce allocations if possible
        m_i420 = webrtc::I420Buffer::Copy(
            inFrame.width(),
            inFrame.height(),
            inFrame.ComponentData(0),
            inFrame.ComponentStride(0),
            inFrame.ComponentData(1),
            inFrame.ComponentStride(1),
            inFrame.ComponentData(2),
            inFrame.ComponentStride(2));
    }
}
