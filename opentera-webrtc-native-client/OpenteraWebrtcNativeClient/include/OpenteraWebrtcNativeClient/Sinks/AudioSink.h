#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_AUDIO_SINK_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_AUDIO_SINK_H

#include <api/media_stream_interface.h>

namespace opentera
{

    /**
     * @brief Class that sinks audio data from the WebRTC transport layer and feeds it to the provided callback.
     */
    class AudioSink : public webrtc::AudioTrackSinkInterface
    {
        std::function<void(
                const void* audioData,
                int bitsPerSample,
                int sampleRate,
                size_t numberOfChannels,
                size_t numberOfFrames)> m_onAudioFrameReceived;

    public:
        explicit AudioSink(std::function<void(
                const void* audioData,
                int bitsPerSample,
                int sampleRate,
                size_t numberOfChannels,
                size_t number_of_frames)> onAudioFrameReceived);

        void OnData(const void* audioData,
               int bitsPerSample,
               int sampleRate,
               size_t numberOfChannels,
               size_t numberOfFrames) override;
    };
}

#endif
