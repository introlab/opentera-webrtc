#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_AUDIO_SINK_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_AUDIO_SINK_H

#include <api/media_stream_interface.h>

namespace opentera
{
    using AudioSinkCallback = std::function<
        void(const void* audioData, int bitsPerSample, int sampleRate, size_t numberOfChannels, size_t numberOfFrames)>;

    /**
     * @brief Class that sinks audio data from the WebRTC transport layer and feeds
     * it to the provided callback.
     */
    class AudioSink : public webrtc::AudioTrackSinkInterface
    {
        AudioSinkCallback m_onAudioFrameReceived;

    public:
        explicit AudioSink(AudioSinkCallback onAudioFrameReceived);

        void OnData(
            const void* audioData,
            int bitsPerSample,
            int sampleRate,
            size_t numberOfChannels,
            size_t numberOfFrames) override;
    };
}

#endif
