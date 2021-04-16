#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_AUDIO_SOURCE_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_SOURCES_AUDIO_SOURCE_H

#include <OpenteraWebrtcNativeClient/Configurations/AudioSourceConfiguration.h>
#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>

#include <api/media_stream_interface.h>
#include <api/notifier.h>

#include <mutex>
#include <set>
#include <vector>

namespace opentera
{

    /**
     * @brief Represents an audio source that can be added to a WebRTC call.
     *
     * Pass a shared_ptr to an instance of this to the StreamClient and call sendFrame for each of your audio frame.
     */
    class AudioSource : public webrtc::Notifier<webrtc::AudioSourceInterface>
    {
        AudioSourceConfiguration m_configuration;
        int m_bitsPerSample;
        int m_sampleRate;
        size_t m_numberOfChannels;
        size_t m_bytesPerFrame;

        std::recursive_mutex m_sinkMutex;
        std::set<webrtc::AudioTrackSinkInterface*> m_sinks;

        size_t m_dataIndex;
        std::vector<uint8_t> m_data; // 10 ms audio frame
        size_t m_dataNumberOfFrames;

    public:
        AudioSource(AudioSourceConfiguration configuration, int bitsPerSample, int sampleRate, size_t numberOfChannels);

        DECLARE_NOT_COPYABLE(AudioSource);
        DECLARE_NOT_MOVABLE(AudioSource);

        void AddSink(webrtc::AudioTrackSinkInterface* sink) override;
        void RemoveSink(webrtc::AudioTrackSinkInterface* sink) override;

        bool remote() const override;
        SourceState state() const override;
        const cricket::AudioOptions options() const override;

        size_t bytesPerSample() const;
        size_t bytesPerFrame() const;
        void sendFrame(const void* audioData, size_t numberOfFrames);

        // Methods to fake a ref counted object, so the Python binding is easier to make because we can use a shared_ptr
        void AddRef() const override;
        rtc::RefCountReleaseStatus Release() const override;
    };
}

#endif
