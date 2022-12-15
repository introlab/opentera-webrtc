/*
 *  Copyright (C) 2018, 2020 Metrological Group B.V.
 *  Copyright (C) 2018, 2020 Igalia S.L.
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
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/mediastream/libwebrtc/GStreamerVideoDecoderFactory.cpp.auto.html
 */

#include "GStreamerVideoDecoderFactory.h"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <api/video_codecs/video_decoder.h>
#include <modules/video_coding/include/video_codec_interface.h>
#include <media/base/codec.h>

#include "OpenteraWebrtcNativeClient/Utils/ClassMacro.h"

#include "frames/GStreamerFramesToFromWebrtc.h"
#include "pipeline/GStreamerAppPipeline.h"
#include "utils/gstreamer_helpers.h"

// TODO: Split this file

namespace opentera
{
    namespace internal
    {
        struct InputTimestamps
        {
            uint64_t timestamp;
            int64_t renderTimeMs;
        };

        class GStreamerVideoDecoder : public webrtc::VideoDecoder
        {
        public:
            GStreamerVideoDecoder()
                : m_gstapp{},
                  m_latestSettings{},
                  m_needsKeyframe{true},
                  m_firstBufferPts{GST_CLOCK_TIME_NONE},
                  m_firstBufferDts{GST_CLOCK_TIME_NONE},
                  m_width{0},
                  m_height{0},
                  m_imageReadyCb{nullptr}
            {
            }

            ~GStreamerVideoDecoder() override = default;

            int32_t Release() final
            {
                if (m_gstapp && m_gstapp->pipeline())
                {
                    auto bus = gst::unique_from_ptr(gst_pipeline_get_bus(GST_PIPELINE(m_gstapp->pipeline())));
                    gst_bus_set_sync_handler(bus.get(), nullptr, nullptr, nullptr);

                    m_gstapp.reset();
                }
                return WEBRTC_VIDEO_CODEC_OK;
            }

            int32_t Decode(const webrtc::EncodedImage& inputImage, bool missingFrames, int64_t renderTimeMs) final
            {
                // TODO: This might not work OR be needed, should be revised
                if (m_needsKeyframe)
                {
                    if (inputImage._frameType != webrtc::VideoFrameType::kVideoFrameKey)
                    {
                        GST_ERROR("Waiting for keyframe but got a delta unit... asking for keyframe");
                        return WEBRTC_VIDEO_CODEC_ERROR;
                    }
                    else
                    {
                        m_needsKeyframe = false;
                    }
                }


                if (!m_gstapp)
                {
                    GST_ERROR("No source set, can't decode.");
                    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
                }


                // No idea why this is needed, taken from WebKit implementation
                if (!GST_CLOCK_TIME_IS_VALID(m_firstBufferPts))
                {
                    gst::unique_ptr<GstPad> srcpad =
                        gst::unique_from_ptr(gst_element_get_static_pad(m_gstapp->src(), "src"));
                    m_firstBufferPts = (static_cast<guint64>(renderTimeMs)) * GST_MSECOND;
                    m_firstBufferDts = (static_cast<guint64>(inputImage.Timestamp())) * GST_MSECOND;
                }

                // FIXME- Use a GstBufferPool.
                m_buffer = gst::unique_from_ptr(
                    gst_buffer_new_wrapped(g_memdup(inputImage.data(), inputImage.size()), inputImage.size()));

                GST_BUFFER_DTS(m_buffer.get()) =
                    (static_cast<guint64>(inputImage.Timestamp()) * GST_MSECOND) - m_firstBufferDts;
                GST_BUFFER_PTS(m_buffer.get()) = (static_cast<guint64>(renderTimeMs) * GST_MSECOND) - m_firstBufferPts;
                internal::InputTimestamps timestamps = {inputImage.Timestamp(), renderTimeMs};
                m_dtsPtsMap[GST_BUFFER_PTS(m_buffer.get())] = timestamps;

                auto sample =
                    gst::unique_from_ptr(gst_sample_new(m_buffer.get(), GetCapsForFrame(inputImage), nullptr, nullptr));

                /** Uncomment to help debugging */
                // GST_WARNING("Pushing sample: %" GST_PTR_FORMAT, sample.get());
                // GST_WARNING("Width: %d, Height: %d, Size: %lu", m_width, m_height,
                // gst_buffer_get_size(m_buffer.get()));

                switch (gst_app_src_push_sample(GST_APP_SRC(m_gstapp->src()), sample.get()))
                {
                    case GST_FLOW_OK:
                        break;
                    case GST_FLOW_FLUSHING:
                        return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
                    default:
                        return WEBRTC_VIDEO_CODEC_ERROR;
                }

                /** Uncomment to help debugging */
                // GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(
                //     GST_BIN(m_gstapp->pipeline()),
                //     GST_DEBUG_GRAPH_SHOW_ALL,
                //     "pipeline-push-sample");
                // std::cout << "Sample (push) is " << std::hex << sample.get() << std::dec << std::endl;

                return pullSample();
            }

            int32_t pullSample()
            {
                GstState state;
                GstState pending;
                gst_element_get_state(m_gstapp->sink(), &state, &pending, GST_SECOND / 100);

                /** Uncomment to help debugging */
                // GST_ERROR("State: %s; Pending: %s", gst_element_state_get_name(state),
                // gst_element_state_get_name(pending));

                /** Uncomment to help debugging */
                // fmt::print("Pulling sample from sink: {}\n", fmt::ptr(m_gstapp->sink()));
                // GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(
                //     GST_BIN(m_gstapp->pipeline()),
                //     GST_DEBUG_GRAPH_SHOW_ALL,
                //     "pipeline-pull-sample");

                auto sample = gst::unique_from_ptr(
                    gst_app_sink_try_pull_sample(GST_APP_SINK(m_gstapp->sink()), GST_SECOND / 100));

                if (!sample)
                {
                    if (!m_gstapp->ready() || state != GST_STATE_PLAYING)
                    {
                        GST_ERROR("Not ready");
                        return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
                    }
                    else
                    {
                        GST_ERROR("Needs more data");
                        return WEBRTC_VIDEO_CODEC_OK;
                    }
                }

                m_gstapp->setReady(true);

                /** Uncomment to help debugging */
                // std::cout << "Sample (pull) is " << std::hex << sample.get() << std::dec << std::endl;

                auto buffer = gst_sample_get_buffer(sample.get());

                // Make sure that the frame.timestamp == previsouly input_frame._timeStamp
                // as it is required by the VideoDecoder baseclass.
                // Taken from the WebKit implementation
                auto timestamps = m_dtsPtsMap[GST_BUFFER_PTS(buffer)];
                m_dtsPtsMap.erase(GST_BUFFER_PTS(buffer));

                /** Uncomment to help debugging */
                // GST_WARNING("Pulling sample: %" GST_PTR_FORMAT, sample.get());
                // GST_WARNING("With size: %lu", gst_buffer_get_size(buffer));
                // GstVideoInfo info;
                // gst_video_info_from_caps(&info, gst_sample_get_caps(sample.get()));
                // GST_VIDEO_INFO_FORMAT(&info);

                // FIXME: Will be unrefed by the frame. Make the frame take a gst::unique_ptr to clarify that it takes
                // ownership?
                gst_sample_ref(sample.get());
                auto frame = LibWebRTCVideoFrameFromGStreamerSample(
                    sample.get(),
                    webrtc::kVideoRotation_0,
                    static_cast<int64_t>(timestamps.timestamp),
                    timestamps.renderTimeMs);

                /** Uncomment to help debugging */
                // GST_LOG_OBJECT(
                //     m_gstapp->pipeline(),
                //     "Output decoded frame! %d -> %" GST_PTR_FORMAT,
                //     frame.timestamp(),
                //     buffer);

                if (m_imageReadyCb)
                {
                    m_imageReadyCb->Decoded(frame, absl::optional<int32_t>(), absl::optional<uint8_t>());
                }

                return WEBRTC_VIDEO_CODEC_OK;
            }

            GstCaps* GetCapsForFrame(const webrtc::EncodedImage& image)
            {
                m_width = image._encodedWidth != 0 ? image._encodedWidth : m_width;
                m_height = image._encodedHeight != 0 ? image._encodedHeight : m_height;
                if (!m_caps)
                {
                    m_caps = gst::unique_from_ptr(gst_caps_new_simple(
                        Caps(),
                        "width",
                        G_TYPE_INT,
                        m_width,
                        "height",
                        G_TYPE_INT,
                        m_height,
                        "alignment",
                        G_TYPE_STRING,
                        "au",
                        nullptr));
                }

                return m_caps.get();
            }

            // TODO: Should be dynamic
            /** Uncomment matchings Caps() and Name() as needed based on the decoder used */
            static const gchar* Caps() { return "video/x-h264"; }
            static const gchar* Name() { return cricket::kH264CodecName; }
            // static const gchar* Caps() { return "video/x-vp8"; }
            // static const gchar* Name() { return cricket::kVp8CodecName; }
            // static const gchar* Caps() { return "video/x-vp9"; }
            // static const gchar* Name() { return cricket::kVp9CodecName; }

            static webrtc::SdpVideoFormat Format() { return webrtc::SdpVideoFormat(Name()); }

            webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override
            {
                webrtc::VideoDecoder::DecoderInfo info;
                info.is_hardware_accelerated = true;
                info.implementation_name = "GStreamer";
                return info;
            }

            bool Configure(const webrtc::VideoDecoder::Settings& settings) override
            {
                // TODO: check all the settings that if changed would require a reconfigure of the pipeline, and only
                // reconfigure if needed
                if (!m_latestSettings || (m_latestSettings->codec_type() != settings.codec_type()))
                {
                    m_gstapp = std::make_unique<internal::GStreamerAppPipeline>();
                    m_latestSettings = settings;
                    return (WEBRTC_VIDEO_CODEC_OK == m_gstapp->init(Caps()));
                }
                else
                {
                    return WEBRTC_VIDEO_CODEC_OK;
                }
            }

            int32_t RegisterDecodeCompleteCallback(webrtc::DecodedImageCallback* callback) final
            {
                m_imageReadyCb = callback;
                return WEBRTC_VIDEO_CODEC_OK;
            }


            DECLARE_NOT_COPYABLE(GStreamerVideoDecoder);
            DECLARE_NOT_MOVABLE(GStreamerVideoDecoder);

        private:
            // TODO: dependency injection for the pipeline to choose encoder/decoder and codec
            std::unique_ptr<internal::GStreamerAppPipeline> m_gstapp;
            std::optional<webrtc::VideoDecoder::Settings> m_latestSettings;
            // TODO: keyframe handling
            bool m_needsKeyframe;
            GstClockTime m_firstBufferPts;
            GstClockTime m_firstBufferDts;
            std::map<GstClockTime, internal::InputTimestamps> m_dtsPtsMap;
            gint m_width;
            gint m_height;
            gst::unique_ptr<GstCaps> m_caps;
            webrtc::DecodedImageCallback* m_imageReadyCb;
            gst::unique_ptr<GstBuffer> m_buffer;
        };
    }

    GStreamerVideoDecoderFactory::GStreamerVideoDecoderFactory() = default;

    std::unique_ptr<webrtc::VideoDecoder>
        GStreamerVideoDecoderFactory::CreateVideoDecoder(const webrtc::SdpVideoFormat& format)
    {
        // TODO: create a decoder based on the format
        // TODO: check if the format is supported, and return {} if not
        return std::make_unique<internal::GStreamerVideoDecoder>();
    }
    std::vector<webrtc::SdpVideoFormat> GStreamerVideoDecoderFactory::GetSupportedFormats() const
    {
        // TODO: dynamic list of supported formats based on what is available in the system
        return {
            internal::GStreamerVideoDecoder::Format(),
        };
    }
}
