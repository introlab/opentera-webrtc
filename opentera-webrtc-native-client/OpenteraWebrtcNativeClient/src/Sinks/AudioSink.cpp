#include <OpenteraWebrtcNativeClient/Sinks/AudioSink.h>

#include <utility>

using namespace std;
using namespace introlab;

/**
 * @brief Construct an AudioStream object
 * @param onAudioDataReceived callback function to consume audio data received on the WebRTC transport layer
 */
AudioSink::AudioSink(function<void(
        const void* audio_data,
        int bits_per_sample,
        int sample_rate,
        size_t number_of_channels,
        size_t number_of_frames)> onAudioDataReceived) : m_onAudioDataReceived(move(onAudioDataReceived))
{

}

/**
 * @brief Called by the WebRTC transport layer when audio data is available
 *
 * @param audio_data audio data buffer
 * @param bits_per_sample number of bits per audio sample
 * @param sample_rate sample rate of the audio data
 * @param number_of_channels number of channel of the audio data
 * @param number_of_frames number of audio frame in the received data
 */
void AudioSink::OnData(
        const void* audio_data,
        int bits_per_sample,
        int sample_rate,
        size_t number_of_channels,
        size_t number_of_frames)
{
    m_onAudioDataReceived(audio_data, bits_per_sample, sample_rate, number_of_channels, number_of_frames);
}