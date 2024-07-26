#ifndef ALSA_PCM_DEVICE_H
#define ALSA_PCM_DEVICE_H

#include <alsa/asoundlib.h>

#include <memory>
#include <string>
#include <cstring>

enum class PcmAudioFrameFormat : std::size_t
{
    Signed8 = 1,
    Signed16 = 2,
    Signed24 = 3,
    SignedPadded24 = 4 + 64,
    Signed32 = 4,

    Unsigned8 = 1 + 16,
    Unsigned16 = 2 + 16,
    Unsigned24 = 3 + 16,
    UnsignedPadded24 = 4 + 16 + 64,
    Unsigned32 = 4 + 16,

    Float = 4 + 32,
    Double = 8 + 32
};

inline std::size_t formatSize(PcmAudioFrameFormat format)
{
    return static_cast<std::size_t>(format) & 0b1111;
}

inline std::size_t pcmFrameSize(PcmAudioFrameFormat format, std::size_t channelCount, std::size_t sampleCount)
{
    return channelCount * sampleCount * formatSize(format);
}

class PcmAudioFrame
{
    PcmAudioFrameFormat m_format;
    std::size_t m_channelCount;
    std::size_t m_sampleCount;
    uint8_t* m_data;
    bool m_hasOwnership;

public:
    PcmAudioFrame(PcmAudioFrameFormat format, std::size_t channelCount, std::size_t sampleCount);
    PcmAudioFrame(PcmAudioFrameFormat format, std::size_t channelCount, std::size_t sampleCount, uint8_t* data);

    PcmAudioFrame(const PcmAudioFrame& other);
    PcmAudioFrame(PcmAudioFrame&& other);
    ~PcmAudioFrame();

    PcmAudioFrameFormat format() const;
    std::size_t channelCount() const;
    std::size_t sampleCount() const;

    uint8_t* data();
    const uint8_t* data() const;
    std::size_t size() const;

    bool hasOwnership() const;

    PcmAudioFrame& operator=(const PcmAudioFrame& other);
    PcmAudioFrame& operator=(PcmAudioFrame&& other);

    uint8_t& operator[](std::size_t i);
    uint8_t operator[](std::size_t i) const;

    void clear();
};

inline PcmAudioFrameFormat PcmAudioFrame::format() const
{
    return m_format;
}

inline std::size_t PcmAudioFrame::channelCount() const
{
    return m_channelCount;
}

inline std::size_t PcmAudioFrame::sampleCount() const
{
    return m_sampleCount;
}

inline uint8_t* PcmAudioFrame::data()
{
    return m_data;
}

inline const uint8_t* PcmAudioFrame::data() const
{
    return m_data;
}

inline std::size_t PcmAudioFrame::size() const
{
    return pcmFrameSize(m_format, m_channelCount, m_sampleCount);
}

inline bool PcmAudioFrame::hasOwnership() const
{
    return m_hasOwnership;
}

inline uint8_t& PcmAudioFrame::operator[](std::size_t i)
{
    return m_data[i];
}

inline uint8_t PcmAudioFrame::operator[](std::size_t i) const
{
    return m_data[i];
}

inline void PcmAudioFrame::clear()
{
    std::memset(m_data, 0, size());
}

class AlsaPcmDevice
{
public:
    enum class Stream
    {
        Playback = SND_PCM_STREAM_PLAYBACK,
        Capture = SND_PCM_STREAM_CAPTURE
    };

private:
    struct PcmDeleter
    {
        void operator()(snd_pcm_t* handle) { snd_pcm_close(handle); }
    };

    PcmAudioFrameFormat m_format;
    std::size_t m_channelCount;
    std::size_t m_frameSampleCount;

    std::unique_ptr<snd_pcm_t, PcmDeleter> m_pcmHandle;

public:
    AlsaPcmDevice(
        const std::string& device,
        Stream stream,
        PcmAudioFrameFormat format,
        std::size_t channelCount,
        std::size_t frameSampleCount,
        std::size_t sampleFrequency);

    void read(PcmAudioFrame& frame);
    void write(const PcmAudioFrame& frame);
    void wait();

private:
    static snd_pcm_stream_t convert(Stream stream);
    static snd_pcm_format_t convert(PcmAudioFrameFormat format);
};

#endif
