#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_AUDIO_SINK_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_AUDIO_SINK_H

#include <api/media_stream_interface.h>

namespace introlab {

    /**
     * @brief Sinks audio data from the WebRTC transport layer and feeds it to the provided callback
     *
     * Pass a shared_ptr to an instance of this to the StreamClient in order to received an audio track
     * contained in the stream received by the client.
     */
    class AudioSink : public webrtc::AudioTrackSinkInterface
    {
        std::function<void(
                const void* audio_data,
                int bits_per_sample,
                int sample_rate,
                size_t number_of_channels,
                size_t number_of_frames)> m_onAudioDataReceived;

    public:
        explicit AudioSink(std::function<void(
                const void* audio_data,
                int bits_per_sample,
                int sample_rate,
                size_t number_of_channels,
                size_t number_of_frames)> onAudioDataReceived);

        void OnData(const void* audio_data,
               int bits_per_sample,
               int sample_rate,
               size_t number_of_channels,
               size_t number_of_frames) override;
    };
}

#endif
