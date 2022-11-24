#include "GStreamerVideoDecoderFactory.h"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <api/video_codecs/video_decoder.h>
#include "utils/gstreamer_helpers.h"
#include <memory>
// #include "OpenteraWebrtcNativeClient/Utils/ClassMacro.h"
#include "modules/video_coding/include/video_codec_interface.h"
#include "utils/GStreamerFramesToFromWebrtc.h"
#include <media/base/codec.h>

namespace opentera
{
    class GStreamerVideoDecoder;
    namespace internal
    {
        enum RunLoopSourcePriority
        {
            // RunLoop::dispatch().
            RunLoopDispatcher = 100,

            // RunLoopTimer priority by default. It can be changed with RunLoopTimer::setPriority().
            RunLoopTimer = 0,

            // Garbage collector timers.
            JavascriptTimer = 200,

            // callOnMainThread.
            MainThreadDispatcherTimer = 100,

            // Memory pressure monitor.
            MemoryPressureHandlerTimer = -100,

            // WebCore timers.
            MainThreadSharedTimer = 100,

            // Used for timers that discard resources like backing store, buffers, etc.
            ReleaseUnusedResourcesTimer = 200,

            // Rendering timer in the threaded compositor.
            CompositingThreadUpdateTimer = 110,

            // Layer flush.
            LayerFlushTimer = -100,

            // DisplayRefreshMonitor timer, should have the same value as the LayerFlushTimer.
            DisplayRefreshMonitorTimer = -100,

            // Rendering timer in the main thread when accelerated compositing is not used.
            NonAcceleratedDrawingTimer = 100,

            // Async IO network callbacks.
            AsyncIONetwork = 100,

            // Disk cache read callbacks.
            DiskCacheRead = 100,

            // Disk cache write callbacks.
            DiskCacheWrite = 200,
        };

        static void simpleBusMessageCallback(GstBus*, GstMessage* message, GstBin* pipeline)
        {
            switch (GST_MESSAGE_TYPE(message))
            {
                case GST_MESSAGE_ERROR:
                    GST_ERROR_OBJECT(pipeline, "Got message: %" GST_PTR_FORMAT, message);
                    {
                        std::string dotFileName = std::string(GST_OBJECT_NAME(pipeline)) + "_error";
                        GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(pipeline, GST_DEBUG_GRAPH_SHOW_ALL, dotFileName.c_str());
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

                        std::string dotFileName = std::string(GST_OBJECT_NAME(pipeline)) +
                                                  gst_element_state_get_name(oldState) +
                                                  gst_element_state_get_name(newState);

                        GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(
                            GST_BIN(pipeline),
                            GST_DEBUG_GRAPH_SHOW_ALL,
                            dotFileName.c_str());
                    }
                    break;
                default:
                    break;
            }
        }

        void disconnectSimpleBusMessageCallback(GstElement* pipeline)
        {
            gst::unique_ptr<GstBus> bus = gst::unique_from_ptr(gst_pipeline_get_bus(GST_PIPELINE(pipeline)));
            g_signal_handlers_disconnect_by_func(
                bus.get(),
                reinterpret_cast<gpointer>(simpleBusMessageCallback),
                pipeline);
        }

        void connectSimpleBusMessageCallback(GstElement* pipeline)
        {
            gst::unique_ptr<GstBus> bus = gst::unique_from_ptr(gst_pipeline_get_bus(GST_PIPELINE(pipeline)));
            gst_bus_add_signal_watch_full(bus.get(), RunLoopSourcePriority::RunLoopDispatcher);
            g_signal_connect(bus.get(), "message", G_CALLBACK(simpleBusMessageCallback), pipeline);
        }

        struct InputTimestamps
        {
            uint64_t timestamp;
            int64_t renderTimeMs;
        };

        class GStreamerAppPipeline;
        struct TrucCallback
        {
            GStreamerAppPipeline* app;
            bool* needChose;
        };
        struct TrucPadAddedCallback
        {
            gst::unique_ptr<GstPad>* sinkpad;
            bool* ready;
        };

        class GStreamerAppPipeline
        {
        public:
            GStreamerAppPipeline()
                : m_gst{nullptr, nullptr},
                  m_pipeline{nullptr},
                  //              m_decodebin{nullptr},
                  m_appsrc{nullptr},
                  m_appsink{nullptr},
                  m_width{0},
                  m_height{0},
                  m_trucCallback{this, nullptr},
                  //                  m_capsfilter{nullptr},
                  m_capsfilter_sinkpad{nullptr},
                  m_decoder{nullptr},
                  m_tee{nullptr},
                  m_queue1{nullptr},
                  m_queue2{nullptr},
                  m_autovideosink{nullptr},
                  m_ready{false},
                  m_padCb{&m_capsfilter_sinkpad, &m_ready}
            {
                //            m_pipeline.reset(
                //                //                gst_parse_launch("appsrc name=src ! decodebin name=decode ! appsink
                //                name=sink",
                //                //                gst::out_ptr(m_error)));
                //                gst_parse_launch(
                //                    "appsrc name=src ! decodebin ! \"video/x-raw\" ! appsink name=sink",
                //                    gst::out_ptr(m_error)));
            }

            GstElement* pipeline() { return m_pipeline.get(); }
            GstElement* src() { return m_appsrc.get(); }
            GstElement* sink() { return m_appsink.get(); }
            [[nodiscard]] bool ready() const { return m_ready; }

            //            static void decodebinPadAddedCb(GstElement* pipeline, GstPad* srcpad, TrucPadAddedCallback*
            //            truc)
            //            {
            //                GST_INFO_OBJECT(srcpad, "connecting pad with %" GST_PTR_FORMAT, truc->sinkpad->get());
            //                if (gst_pad_link(srcpad, truc->sinkpad->get()) != GST_PAD_LINK_OK)
            //                    throw std::runtime_error("Could not link pad: assert not reached");
            //                *truc->ready = true;
            //                GST_DEBUG_BIN_TO_DOT_FILE(
            //                    GST_BIN(gst_element_get_parent(pipeline)),
            //                    GST_DEBUG_GRAPH_SHOW_ALL,
            //                    "pipeline");
            //            }

            int32_t init(const char* caps_str, bool* needsKeyFrameOut)
            {
                m_trucCallback.needChose = needsKeyFrameOut;
                m_appsrc = gst::unique_from_ptr(gst_element_factory_make("appsrc", "src"));
                g_object_set(m_appsrc.get(), "is-live", true, nullptr);

                gst::unique_ptr<GstCaps> caps = nullptr;
                //                m_capsfilter = gst::unique_from_ptr(gst_element_factory_make("identity", "identity"));
                m_decoder = gst::unique_from_ptr(gst_element_factory_make("vaapivp9dec", "decode"));
                //                m_decoder = gst::unique_from_ptr(gst_element_factory_make("vaapih264dec", "decode"));
                //                m_decoder = gst::unique_from_ptr(gst_element_factory_make("avdec_h264", "decode"));
                //                m_decoder = gst::unique_from_ptr(gst_element_factory_make("vaapivp8dec", "decode"));

                //                if (codecSettings)
                //                {
                //                    m_width = codecSettings->width;
                //                    m_height = codecSettings->height;
                //                }

                m_pipeline = gst::unique_from_ptr(gst_pipeline_new("pipeline"));
                connectSimpleBusMessageCallback(m_pipeline.get());

                //                m_capsfilter_sinkpad =
                //                gst::unique_from_ptr(gst_element_get_static_pad(m_capsfilter.get(), "sink"));
                m_capsfilter_sinkpad = gst::unique_from_ptr(gst_element_get_static_pad(m_appsink.get(), "sink"));
                //                auto capsfilter_sourcepad =
                //                gst::unique_from_ptr(gst_element_get_static_pad(m_capsfilter.get(), "src")); auto
                //                trucCaps = gst::unique_from_ptr(gst_caps_new_simple(
                //                    "video/x-raw",
                //                    "width",
                //                    G_TYPE_INT,
                //                    m_width,
                //                    "height",
                //                    G_TYPE_INT,
                //                    m_height,
                //                    nullptr));
                //                gst_pad_set_caps(capsfilter_sourcepad.get(), trucCaps.get());
                //                g_object_set(
                //                    m_decoder.get(),
                //                    "caps",
                //                    "video/x-raw(ANY); audio/x-raw(ANY); text/x-raw(ANY); subpicture/x-dvd;
                //                    subpicture/x-dvb; " "subpicture/x-xsub; subpicture/x-pgs; closedcaption/x-cea-608;
                //                    closedcaption/x-cea-708; " "application/x-onvif-metadata", nullptr);
                //                g_signal_connect(m_decoder.get(), "pad-added", G_CALLBACK(decodebinPadAddedCb),
                //                &m_padCb);
                //                    m_capsfilter_sinkpad.get());
                // Make the decoder output "parsed" frames only and let the main decodebin
                // do the real decoding. This allows us to have optimized decoding/rendering
                // happening in the main pipeline.
                //                if (m_requireParse)
                if (false)  // TODO: for h264 they do it
                {
                    caps = gst::unique_from_ptr(gst_caps_new_simple(caps_str, "parsed", G_TYPE_BOOLEAN, TRUE, nullptr));
                    gst::unique_ptr<GstBus> bus =
                        gst::unique_from_ptr(gst_pipeline_get_bus(GST_PIPELINE(m_pipeline.get())));

                    gst_bus_enable_sync_message_emission(bus.get());
                    g_signal_connect(
                        bus.get(),
                        "sync-message::warning",
                        G_CALLBACK(
                            +[](GstBus*, GstMessage* message, TrucCallback* justThis)
                            {
                                gst::unique_ptr<GError> err;

                                switch (GST_MESSAGE_TYPE(message))
                                {
                                    case GST_MESSAGE_WARNING:
                                    {
                                        gst_message_parse_warning(message, gst::out_ptr(err), nullptr);
                                        [[fallthrough]];
                                    }
                                    case GST_MESSAGE_ERROR:
                                    {
                                        if (!err)
                                            gst_message_parse_error(message, gst::out_ptr(err), nullptr);

                                        if (g_error_matches(err.get(), GST_STREAM_ERROR, GST_STREAM_ERROR_DECODE))
                                        {
                                            GST_INFO_OBJECT(
                                                justThis->app->pipeline(),
                                                "--> needs keyframe (%s)",
                                                err->message);
                                            *(justThis->needChose) = true;
                                        }
                                        break;
                                    }
                                    default:
                                        break;
                                }
                            }),
                        &m_trucCallback);
                }
                else
                {
                    /* FIXME - How could we handle missing keyframes case we do not plug parsers ? */
                    caps = gst::unique_from_ptr(gst_caps_new_empty_simple(caps_str));
                }
                //                g_object_set(m_decoder.get(), "caps", caps.get(), nullptr);

                m_appsink = gst::unique_from_ptr(gst_element_factory_make("appsink", "sink"));
                gst_app_sink_set_emit_signals(GST_APP_SINK(m_appsink.get()), true);
                auto sink_pad = gst::unique_from_ptr(gst_element_get_static_pad(m_appsink.get(), "sink"));
                auto caps_sink =
                    gst::unique_from_ptr(gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "I420", nullptr));
                gst_pad_set_caps(sink_pad.get(), caps_sink.get());

                //                g_signal_connect(m_decoder.get(), "pad-added", G_CALLBACK(decodebinPadAddedCb),
                //                &m_padCb);

                m_tee = gst::unique_from_ptr(gst_element_factory_make("tee", "tee"));
                m_queue1 = gst::unique_from_ptr(gst_element_factory_make("queue", "queue1"));
                m_queue2 = gst::unique_from_ptr(gst_element_factory_make("queue", "queue2"));
                m_autovideosink = gst::unique_from_ptr(gst_element_factory_make("fpsdisplaysink", "avsink"));

                gst_bin_add_many(
                    GST_BIN(pipeline()),
                    m_appsrc.get(),
                    m_decoder.get(),
                    //                    m_capsfilter.get(),
                    m_appsink.get(),
                    m_tee.get(),
                    m_queue1.get(),
                    m_queue2.get(),
                    m_autovideosink.get(),
                    nullptr);
                if (!gst_element_link(m_appsrc.get(), m_decoder.get()))
                {
                    GST_ERROR_OBJECT(pipeline(), "Could not link src to decoder.");
                    return WEBRTC_VIDEO_CODEC_ERROR;
                }

                //                                if (!gst_element_link(m_decoder.get(), m_appsink.get()))
                //                                {
                //                                    GST_ERROR_OBJECT(pipeline(), "Could not link decoder to sink.");
                //                                    return WEBRTC_VIDEO_CODEC_ERROR;
                //                                }

                if (!gst_element_link(m_decoder.get(), m_tee.get()))
                {
                    GST_ERROR_OBJECT(pipeline(), "Could not link decoder to tee.");
                    return WEBRTC_VIDEO_CODEC_ERROR;
                }
                if (!gst_element_link_many(m_tee.get(), m_queue1.get(), m_appsink.get(), nullptr))
                {
                    GST_ERROR_OBJECT(pipeline(), "Could not tee, queue and appsink.");
                    return WEBRTC_VIDEO_CODEC_ERROR;
                }
                if (!gst_element_link_many(m_tee.get(), m_queue2.get(), m_autovideosink.get(), nullptr))
                //                if (!gst_element_link_many(m_decoder.get(), m_queue2.get(),
                //                m_autovideosink.get(),
                //                        nullptr))
                {
                    GST_ERROR_OBJECT(pipeline(), "Could not tee, queue and autovideosink.");
                    return WEBRTC_VIDEO_CODEC_ERROR;
                }


                m_ready = true;
                GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline()), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");

                //                if (!gst_element_link(decoder.get(), capsfilter.get()))
                //                {
                //                    GST_ERROR_OBJECT(pipeline(), "Could not link decoder to capsfilter.");
                //                    return WEBRTC_VIDEO_CODEC_ERROR;
                //                }

                //                if (!gst_element_link(m_capsfilter.get(), m_appsink.get()))
                //                {
                //                    GST_ERROR_OBJECT(pipeline(), "Could not link capsfilter to sink.");
                //                    return WEBRTC_VIDEO_CODEC_ERROR;
                //                }

                if (gst_element_set_state(pipeline(), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
                {
                    GST_ERROR_OBJECT(pipeline(), "Could not set state to PLAYING.");
                    return WEBRTC_VIDEO_CODEC_ERROR;
                }

                // This is an decoder, everything should happen as fast as possible and not
                // be synced on the clock.
                g_object_set(m_appsink.get(), "sync", false, nullptr);

                return WEBRTC_VIDEO_CODEC_OK;
            }
            //            if (!m_pipeline)
            //            {
            //                using namespace std::literals;
            //                throw std::runtime_error(
            //                    "Failed to create GStreamer pipeline"s + (m_error ? (": "s + m_error->message) : ""));
            //                //                return WEBRTC_VIDEO_CODEC_ERROR;
            //            }
            //            //            m_decodebin =
            //            gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(m_pipeline.get()), "decode"));
            //            //            if (!m_decodebin)
            //            //            {
            //            //                //                throw std::runtime_error("Failed to get decodebin from
            //            GStreamer
            //            //                pipeline"); GST_ERROR_OBJECT(m_pipeline.get(), "Failed to get decodebin from
            //            GStreamer
            //            //                pipeline"); return WEBRTC_VIDEO_CODEC_ERROR;
            //            //            }
            //            m_appsrc = gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(m_pipeline.get()), "src"));
            //            if (!m_appsrc)
            //            {
            //                //                throw std::runtime_error("Failed to get appsrc from GStreamer
            //                pipeline"); GST_ERROR_OBJECT(m_pipeline.get(), "Failed to get appsrc from GStreamer
            //                pipeline"); return WEBRTC_VIDEO_CODEC_ERROR;
            //            }
            //            m_appsink = gst::unique_from_ptr(gst_bin_get_by_name(GST_BIN(m_pipeline.get()), "sink"));
            //            if (!m_appsink)
            //            {
            //                //                throw std::runtime_error("Failed to get appsink from GStreamer
            //                pipeline"); GST_ERROR_OBJECT(m_pipeline.get(), "Failed to get appsink from GStreamer
            //                pipeline"); return WEBRTC_VIDEO_CODEC_ERROR;
            //            }
            //            gst_app_sink_set_emit_signals(GST_APP_SINK(m_appsink.get()), true);
            //            // This is an decoder, everything should happen as fast as possible and not
            //            // be synced on the clock.
            //            g_object_set(m_appsink.get(), "sync", false, nullptr);
            //
            //            if (gst_element_set_state(m_pipeline.get(), GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
            //            {
            //                //                throw std::runtime_error("Failed to set GStreamer pipeline to PLAYING
            //                state"); GST_ERROR_OBJECT(m_pipeline.get(), "Failed to set GStreamer pipeline to PLAYING
            //                state"); return WEBRTC_VIDEO_CODEC_ERROR;
            //            }
            //
            //            return WEBRTC_VIDEO_CODEC_OK;
            //        }

            ~GStreamerAppPipeline() = default;

        private:
            gst::Gst m_gst;
            gst::unique_ptr<GstElement> m_pipeline;
            //        gst::unique_ptr<GstElement> m_decodebin;
            gst::unique_ptr<GstElement> m_appsrc;
            gst::unique_ptr<GstElement> m_appsink;
            gst::unique_ptr<GstElement> m_tee;
            gst::unique_ptr<GstElement> m_queue1;
            gst::unique_ptr<GstElement> m_queue2;
            gst::unique_ptr<GstElement> m_autovideosink;
            //            gst::unique_ptr<GstElement> m_capsfilter;
            gst::unique_ptr<GstElement> m_decoder;
            gst::unique_ptr<GstPad> m_capsfilter_sinkpad;
            gst::unique_ptr<GError> m_error;
            gint m_width;
            gint m_height;
            TrucCallback m_trucCallback;
            bool m_ready;
            TrucPadAddedCallback m_padCb;
        };
    }

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

        //        int32_t InitDecode(const webrtc::VideoCodec* codecSettings, int32_t numberOfCores) final
        //        {
        //            m_appsrc = gst::unique_from_ptr(gst_element_factory_make("appsrc", "appsrc"));
        //            auto decoder = gst::unique_from_ptr(gst_element_factory_make("decodebin", "decodebin"));
        //
        //            if (codecSettings)
        //            {
        //                auto m_width = codecSettings->width;
        //                auto m_height = codecSettings->height;
        //            }
        //
        //            m_pipeline = gst::unique_from_ptr(gst_pipeline_new("pipeline"));
        //
        //            auto caps = gst::unique_from_ptr(gst_caps_new_simple(
        //                "video/x-raw",
        //                "format",
        //                G_TYPE_STRING,
        //                "I420",
        //                "width",
        //                G_TYPE_INT,
        //                codecSettings->width,
        //                "height",
        //                G_TYPE_INT,
        //                codecSettings->height,
        //                "framerate",
        //                GST_TYPE_FRACTION,
        //                30,
        //                1,
        //                nullptr));
        //            return WEBRTC_VIDEO_CODEC_OK;
        //        }

        int32_t Release() final
        {
            if (m_gstapp and m_gstapp->pipeline())
            {
                auto bus = gst::unique_from_ptr(gst_pipeline_get_bus(GST_PIPELINE(m_gstapp->pipeline())));
                gst_bus_set_sync_handler(bus.get(), nullptr, nullptr, nullptr);

                //                gst_element_set_state(m_gstapp->pipeline(), GST_STATE_NULL);
                m_gstapp.reset();
            }
            return WEBRTC_VIDEO_CODEC_OK;
        }

        int32_t Decode(const webrtc::EncodedImage& inputImage, bool missingFrames, int64_t renderTimeMs) final
        {
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

                //                else
                //                {
                //                    GST_ERROR("Waiting for keyframe but didn't get full frame, getting out");
                //                    return WEBRTC_VIDEO_CODEC_ERROR;
                //                }
            }


            if (!m_gstapp)
            {
                GST_ERROR("No source set, can't decode.");

                return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
            }
            //            if (!m_ready)
            //            {
            //                GstState currentState;
            //                gst_element_get_state(m_gstapp->pipeline(), &currentState, nullptr, GST_CLOCK_TIME_NONE);
            //                if (currentState != GST_STATE_PLAYING)
            //                {
            //                    GST_ERROR("Not in PLAYING state, can't decode.");
            //                    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
            //                }
            //            }

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

            GST_LOG_OBJECT(
                m_gstapp->pipeline(),
                "%" G_GINT64_FORMAT " Decoding: %" GST_PTR_FORMAT,
                renderTimeMs,
                m_buffer.get());
            auto sample =
                gst::unique_from_ptr(gst_sample_new(m_buffer.get(), GetCapsForFrame(inputImage), nullptr, nullptr));
            //            GST_WARNING("Pushing sample: %" GST_PTR_FORMAT, sample.get());
            //            GST_WARNING("Width: %d, Height: %d, Size: %lu", m_width, m_height,
            //            gst_buffer_get_size(m_buffer.get()));
            switch (gst_app_src_push_sample(GST_APP_SRC(m_gstapp->src()), sample.get()))
            {
                case GST_FLOW_OK:
                    //                    if (!m_gstapp->ready())
                    //                        return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
                    break;
                case GST_FLOW_FLUSHING:
                    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
                default:
                    return WEBRTC_VIDEO_CODEC_ERROR;
            }

            //            return WEBRTC_VIDEO_CODEC_OK;
            return pullSample();
        }

        int32_t pullSample()
        {
            auto sample = gst_app_sink_try_pull_sample(GST_APP_SINK(m_gstapp->sink()), GST_SECOND / 100);
            if (!sample)
            {
                GST_ERROR("Needs more data");
                return WEBRTC_VIDEO_CODEC_OK;
            }
            if (!m_gstapp->ready())
                return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
            auto buffer = gst_sample_get_buffer(sample);

            // Make sure that the frame.timestamp == previsouly input_frame._timeStamp
            // as it is required by the VideoDecoder baseclass.
            auto timestamps = m_dtsPtsMap[GST_BUFFER_PTS(buffer)];
            m_dtsPtsMap.erase(GST_BUFFER_PTS(buffer));

            GST_WARNING("Pulling sample: %" GST_PTR_FORMAT, sample);
            //            GST_WARNING("With size: %lu", gst_buffer_get_size(buffer));

            //            GstVideoInfo info;

            //            gst_video_info_from_caps(&info, gst_sample_get_caps(sample));
            //            GST_WARNING(
            //                "Sample infos: %d, %d, &lu, %",
            //                GST_VIDEO_INFO_WIDTH(&info),
            //                GST_VIDEO_INFO_HEIGHT(&info) GST_VIDEO_INFO_SIZE(&info));
            //            GST_VIDEO_INFO_FORMAT(&info);


            auto frame(LibWebRTCVideoFrameFromGStreamerSample(
                sample,
                webrtc::kVideoRotation_0,
                static_cast<int64_t>(timestamps.timestamp),
                timestamps.renderTimeMs));

            GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
            GST_LOG_OBJECT(
                m_gstapp->pipeline(),
                "Output decoded frame! %d -> %" GST_PTR_FORMAT,
                frame.timestamp(),
                buffer);
            //            GST_INFO("Address is: %" GST_PTR_FORMAT, frame.video_frame_buffer().get());

            if (m_imageReadyCb)
            {
                m_imageReadyCb->Decoded(frame, absl::optional<int32_t>(), absl::optional<uint8_t>());
            }
            //            GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(m_gstapp->pipeline()), GST_DEBUG_GRAPH_SHOW_ALL,
            //            "pipeline_loin");

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
        //        static const gchar* Caps() { return "video/x-h264"; }
        static const gchar* Caps() { return "video/x-vp9"; }
        static const gchar* Name() { return cricket::kH264CodecName; }
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
            // TODO: check the right settings that if changed would require a reconfigure of the pipeline.
            if (!m_latestSettings or (m_latestSettings->codec_type() != settings.codec_type()))
            {
                m_gstapp = std::make_unique<internal::GStreamerAppPipeline>();
                //                m_needsKeyframe = true;
                m_latestSettings = settings;
                return (WEBRTC_VIDEO_CODEC_OK == m_gstapp->init(Caps(), &m_needsKeyframe));
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


        //        DECLARE_NOT_COPYABLE(GStreamerVideoDecoder);
        //        DECLARE_NOT_MOVABLE(GStreamerVideoDecoder);

    private:
        //        gst::unique_ptr<GstElement> m_pipeline;
        std::unique_ptr<internal::GStreamerAppPipeline> m_gstapp;
        std::optional<webrtc::VideoDecoder::Settings> m_latestSettings;
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

    GStreamerVideoDecoderFactory::GStreamerVideoDecoderFactory() = default;

    std::unique_ptr<webrtc::VideoDecoder>
        GStreamerVideoDecoderFactory::CreateVideoDecoder(const webrtc::SdpVideoFormat& format)
    {
        return std::make_unique<GStreamerVideoDecoder>();
    }
    std::vector<webrtc::SdpVideoFormat> GStreamerVideoDecoderFactory::GetSupportedFormats() const
    {
        return {
            //            GStreamerVideoDecoder::Format(),
            //            webrtc::SdpVideoFormat(cricket::kVp8CodecName),
            webrtc::SdpVideoFormat(cricket::kVp9CodecName),
            //            webrtc::SdpVideoFormat(cricket::kH264CodecName),
            //            webrtc::SdpVideoFormat(cricket::kAv1CodecName),
            //            //            webrtc::SdpVideoFormat(cricket::kH265CodecName),
            //            webrtc::SdpVideoFormat(cricket::kRedCodecName),
            //            webrtc::SdpVideoFormat(cricket::kUlpfecCodecName),
            //            webrtc::SdpVideoFormat(cricket::kFlexfecCodecName),
            //            webrtc::SdpVideoFormat(cricket::kRtxCodecName),
        };
    }
}
