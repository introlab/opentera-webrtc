#include <OpenteraWebrtcNativeClient/OpenteraAudioDeviceModule.h>
#include <OpenteraWebrtcNativeClient/Utils/thread.h>

using namespace opentera;
using namespace std;

OpenteraAudioDeviceModule::OpenteraAudioDeviceModule()
    : m_isPlayoutInitialized(false),
      m_isRecordingInitialized(false),
      m_isSpeakerInitialized(false),
      m_isMicrophoneInitialized(false),
      m_isPlaying(false),
      m_isRecording(false),
      m_stopped(true),
      m_audioTransport(nullptr)
{
}

OpenteraAudioDeviceModule::~OpenteraAudioDeviceModule()
{
    stop();
}

void OpenteraAudioDeviceModule::setOnMixedAudioFrameReceived(const AudioSinkCallback& onMixedAudioFrameReceived)
{
    lock_guard<mutex> lock(m_setCallbackMutex);
    stop();
    m_onMixedAudioFrameReceived = onMixedAudioFrameReceived;
    startIfStoppedAndTransportValid();
}

void OpenteraAudioDeviceModule::sendFrame(
    const void* audioData,
    int bitsPerSample,
    int sampleRate,
    size_t numberOfChannels,
    size_t numberOfFrames,
    uint32_t audioDelayMs,
    bool isTyping)
{
    lock_guard<mutex> audioTransportCaptureLock(m_audioTransportCaptureMutex);
    if (m_audioTransport == nullptr)
    {
        return;
    }

    uint32_t newMicLevel = 0;
    m_audioTransport->RecordedDataIsAvailable(
        audioData,
        numberOfFrames,
        bitsPerSample / 8,
        numberOfChannels,
        sampleRate,
        audioDelayMs,
        0,  // Clock drift (not used)
        0,  // Volume (not used)
        isTyping,  // key_pressed
        newMicLevel);  // New mic volume (not used)
}

int32_t OpenteraAudioDeviceModule::ActiveAudioLayer(AudioLayer* audioLayer) const
{
    return 0;
}

int32_t OpenteraAudioDeviceModule::RegisterAudioCallback(webrtc::AudioTransport* audioTransport)
{
    lock_guard<mutex> setCallbackLock(m_setCallbackMutex);
    lock_guard<mutex> audioTransportCaptureLock(m_audioTransportCaptureMutex);
    stop();
    m_audioTransport = audioTransport;
    startIfStoppedAndTransportValid();
    return 0;
}

int32_t OpenteraAudioDeviceModule::Init()
{
    return 0;
}

int32_t OpenteraAudioDeviceModule::Terminate()
{
    return 0;
}

bool OpenteraAudioDeviceModule::Initialized() const
{
    return true;
}

int16_t OpenteraAudioDeviceModule::PlayoutDevices()
{
    return 1;
}

int16_t OpenteraAudioDeviceModule::RecordingDevices()
{
    return 0;
}

int32_t OpenteraAudioDeviceModule::PlayoutDeviceName(
    uint16_t index,
    char name[webrtc::kAdmMaxDeviceNameSize],
    char guid[webrtc::kAdmMaxGuidSize])
{
    strcpy(name, "AudioSink");
    strcpy(guid, "0");
    return 0;
}

int32_t OpenteraAudioDeviceModule::RecordingDeviceName(
    uint16_t index,
    char name[webrtc::kAdmMaxDeviceNameSize],
    char guid[webrtc::kAdmMaxGuidSize])
{
    return 0;
}

int32_t OpenteraAudioDeviceModule::SetPlayoutDevice(uint16_t index)
{
    if (m_isPlayoutInitialized)
    {
        return -1;
    }
    return 0;
}

int32_t OpenteraAudioDeviceModule::SetPlayoutDevice(WindowsDeviceType device)
{
    if (m_isPlayoutInitialized)
    {
        return -1;
    }

    return 0;
}

int32_t OpenteraAudioDeviceModule::SetRecordingDevice(uint16_t index)
{
    if (m_isRecordingInitialized)
    {
        return -1;
    }

    return 0;
}

int32_t OpenteraAudioDeviceModule::SetRecordingDevice(WindowsDeviceType device)
{
    if (m_isRecordingInitialized)
    {
        return -1;
    }

    return 0;
}

int32_t OpenteraAudioDeviceModule::PlayoutIsAvailable(bool* available)
{
    *available = true;
    return 0;
}

int32_t OpenteraAudioDeviceModule::InitPlayout()
{
    m_isPlayoutInitialized = true;
    return 0;
}

bool OpenteraAudioDeviceModule::PlayoutIsInitialized() const
{
    return m_isPlayoutInitialized;
}

int32_t OpenteraAudioDeviceModule::RecordingIsAvailable(bool* available)
{
    *available = true;
    return 0;
}

int32_t OpenteraAudioDeviceModule::InitRecording()
{
    m_isRecordingInitialized = true;
    return 0;
}

bool OpenteraAudioDeviceModule::RecordingIsInitialized() const
{
    return m_isRecordingInitialized;
}

int32_t OpenteraAudioDeviceModule::StartPlayout()
{
    if (!m_isPlayoutInitialized)
    {
        return -1;
    }
    m_isPlaying = true;
    return 0;
}

int32_t OpenteraAudioDeviceModule::StopPlayout()
{
    m_isPlaying = false;
    return 0;
}

bool OpenteraAudioDeviceModule::Playing() const
{
    return m_isPlaying;
}

int32_t OpenteraAudioDeviceModule::StartRecording()
{
    if (!m_isRecordingInitialized)
    {
        return -1;
    }
    m_isRecording = true;
    return 0;
}

int32_t OpenteraAudioDeviceModule::StopRecording()
{
    m_isRecording = false;
    return 0;
}

bool OpenteraAudioDeviceModule::Recording() const
{
    return m_isRecording;
}

int32_t OpenteraAudioDeviceModule::InitSpeaker()
{
    m_isSpeakerInitialized = true;
    return 0;
}

bool OpenteraAudioDeviceModule::SpeakerIsInitialized() const
{
    return m_isSpeakerInitialized;
}

int32_t OpenteraAudioDeviceModule::InitMicrophone()
{
    m_isMicrophoneInitialized = true;
    return 0;
}

bool OpenteraAudioDeviceModule::MicrophoneIsInitialized() const
{
    return m_isMicrophoneInitialized;
}

int32_t OpenteraAudioDeviceModule::SpeakerVolumeIsAvailable(bool* available)
{
    *available = false;
    return 0;
}

int32_t OpenteraAudioDeviceModule::SetSpeakerVolume(uint32_t volume)
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::SpeakerVolume(uint32_t* volume) const
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::MaxSpeakerVolume(uint32_t* maxVolume) const
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::MinSpeakerVolume(uint32_t* minVolume) const
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::MicrophoneVolumeIsAvailable(bool* available)
{
    *available = false;
    return 0;
}

int32_t OpenteraAudioDeviceModule::SetMicrophoneVolume(uint32_t volume)
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::MicrophoneVolume(uint32_t* volume) const
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::MaxMicrophoneVolume(uint32_t* maxVolume) const
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::MinMicrophoneVolume(uint32_t* minVolume) const
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::SpeakerMuteIsAvailable(bool* available)
{
    *available = false;
    return 0;
}

int32_t OpenteraAudioDeviceModule::SetSpeakerMute(bool enable)
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::SpeakerMute(bool* enabled) const
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::MicrophoneMuteIsAvailable(bool* available)
{
    *available = false;
    return 0;
}

int32_t OpenteraAudioDeviceModule::SetMicrophoneMute(bool enable)
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::MicrophoneMute(bool* enabled) const
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::StereoPlayoutIsAvailable(bool* available) const
{
    *available = false;
    return 0;
}

int32_t OpenteraAudioDeviceModule::SetStereoPlayout(bool enable)
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::StereoPlayout(bool* enabled) const
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::StereoRecordingIsAvailable(bool* available) const
{
    *available = false;
    return 0;
}

int32_t OpenteraAudioDeviceModule::SetStereoRecording(bool enable)
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::StereoRecording(bool* enabled) const
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::PlayoutDelay(uint16_t* delayMS) const
{
    *delayMS = 0;
    return 0;
}

bool OpenteraAudioDeviceModule::BuiltInAECIsAvailable() const
{
    return false;
}

bool OpenteraAudioDeviceModule::BuiltInAGCIsAvailable() const
{
    return false;
}

bool OpenteraAudioDeviceModule::BuiltInNSIsAvailable() const
{
    return false;
}

int32_t OpenteraAudioDeviceModule::EnableBuiltInAEC(bool enable)
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::EnableBuiltInAGC(bool enable)
{
    return -1;
}

int32_t OpenteraAudioDeviceModule::EnableBuiltInNS(bool enable)
{
    return -1;
}

void OpenteraAudioDeviceModule::stop()
{
    if (!m_stopped.load() && m_thread != nullptr)
    {
        m_stopped.store(true);
        m_thread->join();
        m_thread = nullptr;
    }
}

void OpenteraAudioDeviceModule::startIfStoppedAndTransportValid()
{
    if (m_stopped.load() && m_audioTransport != nullptr)
    {
        m_stopped.store(false);
        m_thread = make_unique<thread>(&OpenteraAudioDeviceModule::run, this);
        setThreadPriority(*m_thread, ThreadPriority::RealTime);
    }
}

void OpenteraAudioDeviceModule::run()
{
    constexpr chrono::nanoseconds FrameDuration = 10ms;

    constexpr size_t NSamples = 480;
    constexpr size_t NBytesPerSample = 2;
    constexpr size_t NChannels = 1;
    constexpr uint32_t SamplesPerSec = 48000;
    size_t nSamplesOut = 0;
    int64_t elapsedTimeMs = -1;
    int64_t ntpTimeMs = -1;
    size_t counter = 0;
    int64_t lastElapsedTime = -1;

    vector<uint8_t> data(NSamples * NBytesPerSample * NChannels, 0);
    auto start = chrono::steady_clock::now();
    while (!m_stopped.load())
    {
        int32_t result = m_audioTransport->NeedMorePlayData(
            NSamples,
            NBytesPerSample,
            NChannels,
            SamplesPerSec,
            data.data(),
            nSamplesOut,
            &elapsedTimeMs,
            &ntpTimeMs);

        if (elapsedTimeMs > -1 && lastElapsedTime == -1)
        {
            start = chrono::steady_clock::now();
            counter = 0;
        }
        ++counter;
        lastElapsedTime = elapsedTimeMs;

        if (result == 0 && elapsedTimeMs != -1 && m_onMixedAudioFrameReceived)
        {
            m_onMixedAudioFrameReceived(data.data(), 8 * NBytesPerSample, SamplesPerSec, NChannels, nSamplesOut);
        }

        if (elapsedTimeMs == -1)
        {
            this_thread::sleep_for(FrameDuration);
        }
        else
        {
            auto sleep_duration = chrono::duration_cast<chrono::milliseconds>(counter * FrameDuration);
            this_thread::sleep_until(start + sleep_duration);
        }
    }
}
