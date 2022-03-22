#include <OpenteraWebrtcNativeClient/Sinks/AudioSink.h>

#include <utility>

using namespace std;
using namespace opentera;

/**
 * @brief Construct an AudioStream object
 * @param onAudioDataReceived callback function to consume audio data received
 * on the WebRTC transport layer
 */
AudioSink::AudioSink(AudioSinkCallback onAudioDataReceived) : m_onAudioFrameReceived(move(onAudioDataReceived)) {}

/**
 * @brief Called by the WebRTC transport layer when audio data is available
 *
 * @param audioData audio data buffer
 * @param bitsPerSample number of bits per audio sample
 * @param sampleRate sample rate of the audio data
 * @param numberOfChannels number of channel of the audio data
 * @param numberOfFrames number of audio frame in the received data
 */
void AudioSink::OnData(
    const void* audioData,
    int bitsPerSample,
    int sampleRate,
    size_t numberOfChannels,
    size_t numberOfFrames)
{
    if (m_onAudioFrameReceived)
    {
        m_onAudioFrameReceived(audioData, bitsPerSample, sampleRate, numberOfChannels, numberOfFrames);
    }
}
