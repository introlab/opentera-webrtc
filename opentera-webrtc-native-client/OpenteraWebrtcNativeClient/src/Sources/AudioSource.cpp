#include <OpenteraWebrtcNativeClient/Sources/AudioSource.h>

using namespace introlab;
using namespace std;

size_t bytesPerFrame(int bitsPerSample, size_t numberOfChannels)
{
    switch (bitsPerSample)
    {
        case 8:
            return sizeof(int8_t) * numberOfChannels;
        case 16:
            return sizeof(int16_t) * numberOfChannels;
        case 32:
            return sizeof(int32_t) * numberOfChannels;
        default:
            throw runtime_error("Invalid bitsPerSample");
    }
}

/**
 * @brief Construct an  AudioSource
 *
 * @param configuration the configuration applied to the audio stream by the audio transport layer
 * @param bitsPerSample the audio stream sample size (8, 16 or 32 bits)
 * @param sampleRate the audio stream sample rate
 * @param numberOfChannels the audio stream channel count
 */
AudioSource::AudioSource(AudioSourceConfiguration configuration,
        int bitsPerSample,
        int sampleRate,
        size_t numberOfChannels) :
        m_configuration(move(configuration)),
        m_bitsPerSample(bitsPerSample),
        m_sampleRate(sampleRate),
        m_numberOfChannels(numberOfChannels),
        m_bytesPerFrame(::bytesPerFrame(bitsPerSample, numberOfChannels)),
        m_dataIndex(0),
        m_data(m_bytesPerFrame * sampleRate / 100, 0),
        m_dataNumberOfFrames(m_data.size() / m_bytesPerFrame)
{
}

/**
 * Add the specified sink to the source
 * @param sink
 */
void AudioSource::AddSink(webrtc::AudioTrackSinkInterface* sink)
{
    lock_guard<recursive_mutex> lock(m_sinkMutex);
    m_sinks.insert(sink);
}

/**
 * Remove the specified sink from the source
 * @param sink
 */
void AudioSource::RemoveSink(webrtc::AudioTrackSinkInterface* sink)
{
    lock_guard<recursive_mutex> lock(m_sinkMutex);
    m_sinks.erase(sink);
}

/**
 * @brief indicates if this source is remote
 * @return always false, the source is local
 */
bool AudioSource::remote() const
{
    return false;
}

/**
 * @brief indicates if this source is live
 * @return always kLive, the source is live
 */
webrtc::MediaSourceInterface::SourceState AudioSource::state() const
{
    return kLive;
}

/**
 * @return the audio source options
 */
const cricket::AudioOptions AudioSource::options() const
{
    return static_cast<cricket::AudioOptions>(m_configuration);
}

/**
 * @return the sample size
 */
size_t AudioSource::bytesPerSample() const
{
    return m_bitsPerSample / 8;
}

/**
 * @return the frame size
 */
size_t AudioSource::bytesPerFrame() const
{
    return m_bytesPerFrame;
}

/**
 * Send an audio frame
 * @param audioData the audio data
 * @param numberOfFrames the number of frames
 */
void AudioSource::sendFrame(const void* audioData, size_t numberOfFrames)
{
    lock_guard<recursive_mutex> lock(m_sinkMutex);

    size_t dataSize = m_bytesPerFrame * numberOfFrames;
    if (m_dataIndex + dataSize >= m_data.size())
    {
        size_t dataToCopySize = m_data.size() - m_dataIndex;
        memcpy(m_data.data() + m_dataIndex, audioData, dataToCopySize);
        m_dataIndex = 0;

        for (auto sinks : m_sinks)
        {
            sinks->OnData(m_data.data(), m_bitsPerSample, m_sampleRate, m_numberOfChannels, m_dataNumberOfFrames);
        }

        size_t remainingNumberOfFrames = (dataSize - dataToCopySize) / m_bytesPerFrame;
        if (remainingNumberOfFrames > 0)
        {
            sendFrame(reinterpret_cast<const uint8_t*>(audioData) + dataToCopySize, remainingNumberOfFrames);
        }
    }
    else
    {
        memcpy(m_data.data() + m_dataIndex, audioData, m_bytesPerFrame * numberOfFrames);
        m_dataIndex += dataSize;
    }
}

void AudioSource::AddRef() const
{
}

rtc::RefCountReleaseStatus AudioSource::Release() const
{
    return rtc::RefCountReleaseStatus::kOtherRefsRemained;
}
