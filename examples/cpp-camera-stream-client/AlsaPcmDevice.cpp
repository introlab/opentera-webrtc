#include "AlsaPcmDevice.h"

#include <iostream>
#include <map>

using namespace std;

PcmAudioFrame::PcmAudioFrame(PcmAudioFrameFormat format, size_t channelCount, size_t sampleCount)
    : m_format(format),
      m_channelCount(channelCount),
      m_sampleCount(sampleCount),
      m_hasOwnership(true)
{
    m_data = new uint8_t[size()];
}

PcmAudioFrame::PcmAudioFrame(PcmAudioFrameFormat format, size_t channelCount, size_t sampleCount, uint8_t* data)
    : m_format(format),
      m_channelCount(channelCount),
      m_sampleCount(sampleCount),
      m_data(data),
      m_hasOwnership(false)
{
}

PcmAudioFrame::PcmAudioFrame(const PcmAudioFrame& other)
    : m_format(other.m_format),
      m_channelCount(other.m_channelCount),
      m_sampleCount(other.m_sampleCount),
      m_hasOwnership(true)
{
    m_data = new uint8_t[size()];
    memcpy(m_data, other.m_data, size());
}

PcmAudioFrame::PcmAudioFrame(PcmAudioFrame&& other)
    : m_format(other.m_format),
      m_channelCount(other.m_channelCount),
      m_sampleCount(other.m_sampleCount),
      m_hasOwnership(other.m_hasOwnership)
{
    m_data = other.m_data;

    other.m_channelCount = 0;
    other.m_sampleCount = 0;
    other.m_data = nullptr;
}

PcmAudioFrame::~PcmAudioFrame()
{
    if (m_data != nullptr && m_hasOwnership)
    {
        delete[] m_data;
    }
}

PcmAudioFrame& PcmAudioFrame::operator=(const PcmAudioFrame& other)
{
    if (m_format != other.m_format || m_channelCount != other.m_channelCount || m_sampleCount != other.m_sampleCount)
    {
        if (m_data != nullptr && m_hasOwnership)
        {
            delete[] m_data;
        }

        m_format = other.m_format;
        m_channelCount = other.m_channelCount;
        m_sampleCount = other.m_sampleCount;
        m_hasOwnership = true;

        m_data = new uint8_t[size()];
    }
    memcpy(m_data, other.m_data, size());

    return *this;
}

PcmAudioFrame& PcmAudioFrame::operator=(PcmAudioFrame&& other)
{
    if (m_data != nullptr && m_hasOwnership)
    {
        delete[] m_data;
    }

    m_format = other.m_format;
    m_channelCount = other.m_channelCount;
    m_sampleCount = other.m_sampleCount;
    m_data = other.m_data;
    m_hasOwnership = other.m_hasOwnership;

    other.m_channelCount = 0;
    other.m_sampleCount = 0;
    other.m_data = nullptr;

    return *this;
}

struct PcmParamsDeleter
{
    void operator()(snd_pcm_hw_params_t* handle) { snd_pcm_hw_params_free(handle); }
};

AlsaPcmDevice::AlsaPcmDevice(
    const std::string& device,
    Stream stream,
    PcmAudioFrameFormat format,
    std::size_t channelCount,
    std::size_t frameSampleCount,
    std::size_t sampleFrequency)
    : m_format(format),
      m_channelCount(channelCount),
      m_frameSampleCount(frameSampleCount)
{
    int err;
    snd_pcm_t* pcmHandlePointer;
    snd_pcm_hw_params_t* paramsPointer;

    if ((err = snd_pcm_open(&pcmHandlePointer, device.c_str(), convert(stream), 0)) < 0)
    {
        cerr << "Cannot open audio device: " << device << "(" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }

    unique_ptr<snd_pcm_t, PcmDeleter> pcmHandle(pcmHandlePointer);

    if ((err = snd_pcm_hw_params_malloc(&paramsPointer)) < 0)
    {
        cerr << "Cannot allocate hardware parameter structure (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }

    unique_ptr<snd_pcm_hw_params_t, PcmParamsDeleter> params(paramsPointer);

    if ((err = snd_pcm_hw_params_any(pcmHandle.get(), params.get())) < 0)
    {
        cerr << "Cannot initialize hardware parameter structure (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }

    if ((err = snd_pcm_hw_params_set_access(pcmHandle.get(), params.get(), SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
    {
        cerr << "Cannot set access type (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }

    if ((err = snd_pcm_hw_params_set_format(pcmHandle.get(), params.get(), convert(format))) < 0)
    {
        cerr << "Cannot set sample format (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }

    if ((err = snd_pcm_hw_params_set_rate(pcmHandle.get(), params.get(), static_cast<int>(sampleFrequency), 0)) < 0)
    {
        cerr << "Cannot set sample rate (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }

    if ((err = snd_pcm_hw_params_set_channels(pcmHandle.get(), params.get(), static_cast<int>(channelCount))) < 0)
    {
        cerr << "Cannot set channel count (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }

    snd_pcm_uframes_t periodSize = static_cast<snd_pcm_uframes_t>(frameSampleCount);
    if ((err = snd_pcm_hw_params_set_period_size(pcmHandle.get(), params.get(), periodSize, 0)) < 0)
    {
        cerr << "Cannot set period size (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }

    if ((err = snd_pcm_hw_params(pcmHandle.get(), params.get())) < 0)
    {
        cerr << "Cannot set parameters (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }

    if ((err = snd_pcm_prepare(pcmHandle.get())) < 0)
    {
        cerr << "Cannot prepare audio interface for use (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }

    m_pcmHandle = move(pcmHandle);
}

void AlsaPcmDevice::read(PcmAudioFrame& frame)
{
    if (frame.format() != m_format || frame.channelCount() != m_channelCount ||
        frame.sampleCount() != m_frameSampleCount)
    {
        cerr << "Invalid format, channelCount or sampleCount" << endl;
        exit(EXIT_FAILURE);
    }

    snd_pcm_uframes_t periodSize = static_cast<snd_pcm_uframes_t>(m_frameSampleCount);
    int err = snd_pcm_readi(m_pcmHandle.get(), frame.data(), periodSize);

    if (err == -EPIPE)
    {
        // EPIPE means overrun
        err = snd_pcm_recover(m_pcmHandle.get(), err, 1);
        if (err >= 0)
        {
            err = snd_pcm_readi(m_pcmHandle.get(), frame.data(), periodSize);
        }
    }

    if (err != periodSize)
    {
        cerr << "Read from audio interface failed (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }
}

void AlsaPcmDevice::write(const PcmAudioFrame& frame)
{
    if (frame.format() != m_format || frame.channelCount() != m_channelCount ||
        frame.sampleCount() != m_frameSampleCount)
    {
        cerr << "Invalid format, channelCount or sampleCount" << endl;
        exit(EXIT_FAILURE);
    }

    snd_pcm_uframes_t periodSize = static_cast<snd_pcm_uframes_t>(m_frameSampleCount);
    int err = snd_pcm_writei(m_pcmHandle.get(), frame.data(), periodSize);

    if (err == -EPIPE)
    {
        // EPIPE means underrun
        err = snd_pcm_recover(m_pcmHandle.get(), err, 1);
        if (err >= 0)
        {
            err = snd_pcm_writei(m_pcmHandle.get(), frame.data(), periodSize);
        }
    }

    if (err != periodSize)
    {
        cerr << "Write to audio interface failed (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }
}

void AlsaPcmDevice::wait()
{
    int err = snd_pcm_wait(m_pcmHandle.get(), -1);
    if (err != 1)
    {
        cerr << "snd_pcm_wait failed (" << err << ": " << snd_strerror(err) << ")" << endl;
        exit(EXIT_FAILURE);
    }
}

snd_pcm_stream_t AlsaPcmDevice::convert(Stream stream)
{
    switch (stream)
    {
        case Stream::Playback:
            return SND_PCM_STREAM_PLAYBACK;
        case Stream::Capture:
            return SND_PCM_STREAM_CAPTURE;
    }

    cerr << "Not supported stream" << endl;
    exit(EXIT_FAILURE);
}

snd_pcm_format_t AlsaPcmDevice::convert(PcmAudioFrameFormat format)
{
    static const map<PcmAudioFrameFormat, snd_pcm_format_t> Mapping(
        {{PcmAudioFrameFormat::Signed8, SND_PCM_FORMAT_S8},
         {PcmAudioFrameFormat::Signed16, SND_PCM_FORMAT_S16_LE},
         {PcmAudioFrameFormat::Signed24, SND_PCM_FORMAT_S24_LE},
         {PcmAudioFrameFormat::Signed32, SND_PCM_FORMAT_S32_LE},

         {PcmAudioFrameFormat::Unsigned8, SND_PCM_FORMAT_U8},
         {PcmAudioFrameFormat::Unsigned16, SND_PCM_FORMAT_U16_LE},
         {PcmAudioFrameFormat::Unsigned24, SND_PCM_FORMAT_U24_LE},
         {PcmAudioFrameFormat::Unsigned32, SND_PCM_FORMAT_U32_LE},

         {PcmAudioFrameFormat::Float, SND_PCM_FORMAT_FLOAT_LE},
         {PcmAudioFrameFormat::Double, SND_PCM_FORMAT_FLOAT64_LE}});

    auto it = Mapping.find(format);
    if (it != Mapping.end())
    {
        return it->second;
    }

    cerr << "Not supported format" << endl;
    exit(EXIT_FAILURE);
}
