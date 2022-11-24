#include "GStreamerFramesToFromWebrtc.h"
#include <gst/video/video.h>
#include <api/video/video_frame.h>
#include <api/video/i420_buffer.h>
#include <common_video/include/video_frame_buffer.h>
#include "OpenteraWebrtcNativeClient/Utils/ClassMacro.h"
#include "gst/gst.h"
#include <thread>

namespace opentera
{
    class GstMappedFrame
    {
    public:
        GstMappedFrame(GstBuffer* buffer, GstVideoInfo info, GstMapFlags flags)
        {
            m_isValid = gst_video_frame_map(&m_frame, &info, buffer, flags);
        }
        DECLARE_NOT_COPYABLE(GstMappedFrame);

        GstMappedFrame(GstSample* sample, GstMapFlags flags)
        {
            GstVideoInfo info;

            if (!gst_video_info_from_caps(&info, gst_sample_get_caps(sample)))
            {
                m_isValid = false;
                return;
            }

            m_isValid = gst_video_frame_map(&m_frame, &info, gst_sample_get_buffer(sample), flags);
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
                gst_video_frame_unmap(&m_frame);
            m_isValid = false;
        }

        explicit operator bool() const { return m_isValid; }

    private:
        GstVideoFrame m_frame;
        bool m_isValid{false};
    };

    gst::unique_ptr<GstSample> GStreamerSampleFromLibWebRTCVideoFrame(const webrtc::VideoFrame& frame)
    {
        //        if (frame.video_frame_buffer()->type() == webrtc::VideoFrameBuffer::Type::kNative)
        //        {
        //            auto framebuffer = dynamic_cast<GStreamerVideoFrameLibWebRTC*>(frame.video_frame_buffer().get());
        //            auto gstsample = framebuffer->getSample();
        //
        //            GST_LOG("Reusing native GStreamer sample: %p", gstsample);
        //
        //            return gstsample;
        //        }

        auto webrtcbuffer = frame.video_frame_buffer()->ToI420().get();
        // FIXME - Check lifetime of those buffers.
        const uint8_t* comps[3] = {webrtcbuffer->DataY(), webrtcbuffer->DataU(), webrtcbuffer->DataV()};

        GstVideoInfo info;
        gst_video_info_set_format(&info, GST_VIDEO_FORMAT_I420, frame.width(), frame.height());
        auto buffer = gst::unique_from_ptr(gst_buffer_new());
        for (gint i = 0; i < 3; i++)
        {
            gsize compsize = GST_VIDEO_INFO_COMP_STRIDE(&info, i) * GST_VIDEO_INFO_COMP_HEIGHT(&info, i);

            GstMemory* comp = gst_memory_new_wrapped(
                GST_MEMORY_FLAG_PHYSICALLY_CONTIGUOUS | GST_MEMORY_FLAG_READONLY,
                const_cast<gpointer>(reinterpret_cast<const void*>(comps[i])),
                compsize,
                0,
                compsize,
                webrtcbuffer,
                nullptr);
            gst_buffer_append_memory(buffer.get(), comp);
        }

        auto caps = gst::unique_from_ptr(gst_video_info_to_caps(&info));
        auto sample = gst::unique_from_ptr(gst_sample_new(buffer.get(), caps.get(), nullptr, nullptr));
        return sample;
    }
    webrtc::VideoFrame LibWebRTCVideoFrameFromGStreamerSample(
        GstSample* sample,
        webrtc::VideoRotation rotation,
        int64_t timestamp,
        int64_t renderTimeMs)
    {
        auto frameBuffer(GStreamerVideoFrameLibWebRTC::create(sample));

        auto builder = webrtc::VideoFrame::Builder{};
        builder.set_video_frame_buffer(frameBuffer)
            .set_timestamp_rtp(timestamp)
            .set_timestamp_ms(renderTimeMs)
            .set_rotation(rotation);

        return builder.build();
        //        return std::make_unique<webrtc::VideoFrame>(frameBuffer, timestamp, renderTimeMs, rotation);
    }

    rtc::scoped_refptr<webrtc::VideoFrameBuffer> GStreamerVideoFrameLibWebRTC::create(GstSample* sample)
    {
        GstVideoInfo info;

        if (!gst_video_info_from_caps(&info, gst_sample_get_caps(sample)))
            throw std::runtime_error("Never reached");
        //        ASSERT_NOT_REACHED();
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
            //            return;
            //                return nullptr;
        }

        m_i420 = m_bufferPool.CreateI420Buffer(inFrame.width(), inFrame.height());
        //        ASSERT(newBuffer);
        if (!m_i420)
        {
            GST_WARNING("RealtimeOutgoingVideoSourceGStreamer::videoSampleAvailable unable to allocate buffer for "
                        "conversion to YUV");
            throw std::runtime_error(
                "RealtimeOutgoingVideoSourceGStreamer::videoSampleAvailable unable to allocate buffer for "
                "conversion to YUV");
            //            return;
            //                return nullptr;
        }

        if (inFrame.format() == GST_VIDEO_FORMAT_I420)
        {
            m_i420->Copy(
                inFrame.width(),
                inFrame.height(),
                inFrame.ComponentData(0),
                inFrame.ComponentStride(0),
                inFrame.ComponentData(1),
                inFrame.ComponentStride(1),
                inFrame.ComponentData(2),
                inFrame.ComponentStride(2));

            //            return newBuffer;
            return;
        }

        GstVideoInfo outInfo;

        gst_video_info_set_format(&outInfo, GST_VIDEO_FORMAT_I420, inFrame.width(), inFrame.height());
        auto info = inFrame.info();
        outInfo.fps_n = info->fps_n;
        outInfo.fps_d = info->fps_d;

        gst::unique_ptr<GstBuffer> buffer = gst::unique_from_ptr(gst_buffer_new_wrapped_full(
            GST_MEMORY_FLAG_NO_SHARE,
            m_i420->MutableDataY(),
            outInfo.size,
            0,
            outInfo.size,
            nullptr,
            nullptr));

        GstMappedFrame outFrame(buffer.get(), outInfo, GST_MAP_WRITE);

        gst::unique_ptr<GstVideoConverter> videoConverter(gst_video_converter_new(
            inFrame.info(),
            &outInfo,
            gst_structure_new(
                "GstVideoConvertConfig",
                GST_VIDEO_CONVERTER_OPT_THREADS,
                G_TYPE_UINT,
                std::max(std::thread::hardware_concurrency(), 1u),
                nullptr)));

        //            ASSERT(videoConverter);
        if (!videoConverter)
        {
            GST_WARNING("RealtimeOutgoingVideoSourceGStreamer::videoSampleAvailable unable to allocate video "
                        "converter for conversion to YUV");
            throw std::runtime_error(
                "RealtimeOutgoingVideoSourceGStreamer::videoSampleAvailable unable to allocate video "
                "converter for conversion to YUV");
            //            return;
            //                    return nullptr;
        }

        gst_video_converter_frame(videoConverter.get(), inFrame.get(), outFrame.get());

        //                return newBuffer;
    }

    rtc::scoped_refptr<webrtc::I420BufferInterface> GStreamerVideoFrameLibWebRTC::ToI420()
    {
        return m_i420;
        //        GstMappedFrame inFrame(m_sample.get(), GST_MAP_READ);
        //
        //        if (!inFrame)
        //        {
        //            GST_WARNING("Could not map frame");
        //
        //            return nullptr;
        //        }
        //
        //        auto newBuffer = m_bufferPool.CreateI420Buffer(inFrame.width(), inFrame.height());
        //        //        ASSERT(newBuffer);
        //        if (!newBuffer)
        //        {
        //            GST_WARNING("RealtimeOutgoingVideoSourceGStreamer::videoSampleAvailable unable to allocate buffer
        //            for "
        //                        "conversion to YUV");
        //            return nullptr;
        //        }
        //
        //        if (inFrame.format() != GST_VIDEO_FORMAT_I420)
        //        {
        //            GstVideoInfo outInfo;
        //
        //            gst_video_info_set_format(&outInfo, GST_VIDEO_FORMAT_I420, inFrame.width(), inFrame.height());
        //            auto info = inFrame.info();
        //            outInfo.fps_n = info->fps_n;
        //            outInfo.fps_d = info->fps_d;
        //
        //            gst::unique_ptr<GstBuffer> buffer = gst::unique_from_ptr(gst_buffer_new_wrapped_full(
        //                GST_MEMORY_FLAG_NO_SHARE,
        //                newBuffer->MutableDataY(),
        //                outInfo.size,
        //                0,
        //                outInfo.size,
        //                nullptr,
        //                nullptr));
        //
        //            GstMappedFrame outFrame(buffer.get(), outInfo, GST_MAP_WRITE);
        //
        //            gst::unique_ptr<GstVideoConverter> videoConverter(gst_video_converter_new(
        //                inFrame.info(),
        //                &outInfo,
        //                gst_structure_new(
        //                    "GstVideoConvertConfig",
        //                    GST_VIDEO_CONVERTER_OPT_THREADS,
        //                    G_TYPE_UINT,
        //                    std::max(std::thread::hardware_concurrency(), 1u),
        //                    nullptr)));
        //
        //            //            ASSERT(videoConverter);
        //            if (!videoConverter)
        //            {
        //                GST_WARNING("RealtimeOutgoingVideoSourceGStreamer::videoSampleAvailable unable to allocate
        //                video "
        //                            "converter for conversion to YUV");
        //                return nullptr;
        //            }
        //
        //            gst_video_converter_frame(videoConverter.get(), inFrame.get(), outFrame.get());
        //
        //            return newBuffer;
        //        }
        //
        //        newBuffer->Copy(
        //            inFrame.width(),
        //            inFrame.height(),
        //            inFrame.ComponentData(0),
        //            inFrame.ComponentStride(0),
        //            inFrame.ComponentData(1),
        //            inFrame.ComponentStride(1),
        //            inFrame.ComponentData(2),
        //            inFrame.ComponentStride(2));
        //
        //        return newBuffer;
    }
}