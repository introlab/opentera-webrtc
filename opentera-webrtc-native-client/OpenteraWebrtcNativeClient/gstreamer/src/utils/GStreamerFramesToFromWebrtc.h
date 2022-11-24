#pragma once
#include "gstreamer_helpers.h"
#include <gst/gst.h>
#include <gst/video/video.h>
#include <memory>
#include <api/frame_transformer_interface.h>
#include <common_video/include/video_frame_buffer_pool.h>
#include "OpenteraWebrtcNativeClient/Utils/ClassMacro.h"

namespace opentera
{
    gst::unique_ptr<GstSample> GStreamerSampleFromLibWebRTCVideoFrame(const webrtc::VideoFrame& frame);
    //    std::unique_ptr<webrtc::VideoFrame> LibWebRTCVideoFrameFromGStreamerSample(
    webrtc::VideoFrame LibWebRTCVideoFrameFromGStreamerSample(
        GstSample* sample,
        webrtc::VideoRotation videoRotation,
        int64_t timestamp,
        int64_t renderTimeMs);

    class GStreamerVideoFrameLibWebRTC : public rtc::RefCountedObject<webrtc::VideoFrameBuffer>
    {
    public:
        GStreamerVideoFrameLibWebRTC(GstSample* sample, GstVideoInfo info)
            : m_sample(gst::shared_from_ptr(sample)),
              m_info(info),
              m_bufferPool{},
              m_i420{}
        {
        }

        //        ~GStreamerVideoFrameLibWebRTC() override { GST_ERROR("Destructeur appelé!"); };

        static rtc::scoped_refptr<webrtc::VideoFrameBuffer> create(GstSample*);

        GstSample* getSample();
        rtc::scoped_refptr<webrtc::I420BufferInterface> ToI420() final;

        const webrtc::I420BufferInterface* GetI420() const final;
        //        { return webrtc::VideoFrameBuffer::GetI420(); }

        int width() const override { return GST_VIDEO_INFO_WIDTH(&m_info); }
        int height() const override { return GST_VIDEO_INFO_HEIGHT(&m_info); }

        auto& refCount() { return ref_count_; };

    private:
        webrtc::VideoFrameBuffer::Type type() const override;

        void createCachedI420() const;

        std::shared_ptr<GstSample> m_sample;
        GstVideoInfo m_info;
        mutable webrtc::VideoFrameBufferPool m_bufferPool;
        mutable rtc::scoped_refptr<webrtc::I420Buffer> m_i420;
    };
}