#include <OpenteraWebrtcNativeClient/BlackHoleAudioCaptureModule.h>
#include <OpenteraWebrtcNativeClient/Utils/thread.h>

using namespace opentera;
using namespace std;

BlackHoleAudioCaptureModule::BlackHoleAudioCaptureModule() :
        m_isPlayoutInitialized(false),
        m_isRecordingInitialized(false),
        m_isSpeakerInitialized(false),
        m_isMicrophoneInitialized(false),
        m_isPlaying(false),
        m_isRecording(false),
        m_stopped(false),
        m_thread(&BlackHoleAudioCaptureModule::run, this),
        m_audioCallback(nullptr)
{
    setThreadPriority(m_thread, ThreadPriority::RealTime);
}

BlackHoleAudioCaptureModule::~BlackHoleAudioCaptureModule()
{
    m_stopped.store(true);
    m_thread.join();
}

void BlackHoleAudioCaptureModule::run()
{
    constexpr chrono::milliseconds SleepDuration = 1ms;

    constexpr size_t NSamples = 480;
    constexpr size_t NBytesPerSample = 2;
    constexpr size_t NChannels = 1;
    constexpr uint32_t SamplesPerSec = 48000;
    size_t nSamplesOut = 0;
    int64_t elapsedTimeMs = -1;
    int64_t ntpTimeMs = -1;

    vector<uint8_t> data(NSamples * NBytesPerSample * NChannels, 0);
    while (!m_stopped.load())
    {
        auto start = chrono::steady_clock::now();

        auto audioCallback = m_audioCallback.load();
        if (audioCallback == nullptr)
        {
            continue;
        }

        audioCallback->NeedMorePlayData(NSamples, NBytesPerSample, NChannels, SamplesPerSec, data.data(), nSamplesOut,
                &elapsedTimeMs, &ntpTimeMs);
        this_thread::sleep_for(SleepDuration);
    }
}

int32_t BlackHoleAudioCaptureModule::ActiveAudioLayer(AudioLayer* audioLayer) const
{
    return 0;
}

int32_t BlackHoleAudioCaptureModule::RegisterAudioCallback(webrtc::AudioTransport* audioCallback)
{
    m_audioCallback.store(audioCallback);
    return 0;
}

int32_t BlackHoleAudioCaptureModule::Init()
{
    return 0;
}

int32_t BlackHoleAudioCaptureModule::Terminate()
{
    return 0;
}

bool BlackHoleAudioCaptureModule::Initialized() const
{
    return true;
}

int16_t BlackHoleAudioCaptureModule::PlayoutDevices()
{
    return 1;
}

int16_t BlackHoleAudioCaptureModule::RecordingDevices()
{
    return 0;
}

int32_t BlackHoleAudioCaptureModule::PlayoutDeviceName(uint16_t index,
        char name[webrtc::kAdmMaxDeviceNameSize],
        char guid[webrtc::kAdmMaxGuidSize])
{
    strcpy(name, "AudioSink");
    strcpy(guid, "0");
    return 0;
}

int32_t BlackHoleAudioCaptureModule::RecordingDeviceName(uint16_t index,
        char name[webrtc::kAdmMaxDeviceNameSize],
        char guid[webrtc::kAdmMaxGuidSize])
{
    return 0;
}

int32_t BlackHoleAudioCaptureModule::SetPlayoutDevice(uint16_t index)
{
    if (m_isPlayoutInitialized)
    {
        return -1;
    }
    return 0;
}

int32_t BlackHoleAudioCaptureModule::SetPlayoutDevice(WindowsDeviceType device)
{
    if (m_isPlayoutInitialized)
    {
        return -1;
    }

    return 0;
}

int32_t BlackHoleAudioCaptureModule::SetRecordingDevice(uint16_t index)
{
    if (m_isRecordingInitialized)
    {
        return -1;
    }

    return 0;
}

int32_t BlackHoleAudioCaptureModule::SetRecordingDevice(WindowsDeviceType device)
{
    if (m_isRecordingInitialized)
    {
        return -1;
    }

    return 0;
}

int32_t BlackHoleAudioCaptureModule::PlayoutIsAvailable(bool* available)
{
    *available = true;
    return 0;
}

int32_t BlackHoleAudioCaptureModule::InitPlayout()
{
    m_isPlayoutInitialized = true;
    return 0;
}

bool BlackHoleAudioCaptureModule::PlayoutIsInitialized() const
{
    return m_isPlayoutInitialized;
}

int32_t BlackHoleAudioCaptureModule::RecordingIsAvailable(bool* available)
{
    *available = true;
    return 0;
}

int32_t BlackHoleAudioCaptureModule::InitRecording()
{
    m_isRecordingInitialized = true;
    return 0;
}

bool BlackHoleAudioCaptureModule::RecordingIsInitialized() const
{
    return m_isRecordingInitialized;
}

int32_t BlackHoleAudioCaptureModule::StartPlayout()
{
    if (!m_isPlayoutInitialized)
    {
        return -1;
    }
    m_isPlaying = true;
    return 0;
}

int32_t BlackHoleAudioCaptureModule::StopPlayout()
{
    m_isPlaying = false;
    return 0;
}

bool BlackHoleAudioCaptureModule::Playing() const
{
    return m_isPlaying;
}

int32_t BlackHoleAudioCaptureModule::StartRecording()
{
    if (!m_isRecordingInitialized)
    {
        return -1;
    }
    m_isRecording = true;
    return 0;
}

int32_t BlackHoleAudioCaptureModule::StopRecording()
{
    m_isRecording = false;
    return 0;
}

bool BlackHoleAudioCaptureModule::Recording() const
{
    return m_isRecording;
}

int32_t BlackHoleAudioCaptureModule::InitSpeaker()
{
    m_isSpeakerInitialized = true;
    return 0;
}

bool BlackHoleAudioCaptureModule::SpeakerIsInitialized() const
{
    return m_isSpeakerInitialized;
}

int32_t BlackHoleAudioCaptureModule::InitMicrophone()
{
    m_isMicrophoneInitialized = true;
    return 0;
}

bool BlackHoleAudioCaptureModule::MicrophoneIsInitialized() const
{
    return m_isMicrophoneInitialized;
}

int32_t BlackHoleAudioCaptureModule::SpeakerVolumeIsAvailable(bool* available)
{
    *available = false;
    return 0;
}

int32_t BlackHoleAudioCaptureModule::SetSpeakerVolume(uint32_t volume)
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::SpeakerVolume(uint32_t* volume) const
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::MaxSpeakerVolume(uint32_t* maxVolume) const
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::MinSpeakerVolume(uint32_t* minVolume) const
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::MicrophoneVolumeIsAvailable(bool* available)
{
    *available = false;
    return 0;
}

int32_t BlackHoleAudioCaptureModule::SetMicrophoneVolume(uint32_t volume)
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::MicrophoneVolume(uint32_t* volume) const
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::MaxMicrophoneVolume(uint32_t* maxVolume) const
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::MinMicrophoneVolume(uint32_t* minVolume) const
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::SpeakerMuteIsAvailable(bool* available)
{
    *available = false;
    return 0;
}

int32_t BlackHoleAudioCaptureModule::SetSpeakerMute(bool enable)
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::SpeakerMute(bool* enabled) const
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::MicrophoneMuteIsAvailable(bool* available)
{
    *available = false;
    return 0;
}

int32_t BlackHoleAudioCaptureModule::SetMicrophoneMute(bool enable)
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::MicrophoneMute(bool* enabled) const
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::StereoPlayoutIsAvailable(bool* available) const
{
    *available = false;
    return 0;
}

int32_t BlackHoleAudioCaptureModule::SetStereoPlayout(bool enable)
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::StereoPlayout(bool* enabled) const
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::StereoRecordingIsAvailable(bool* available) const
{
    *available = false;
    return 0;
}

int32_t BlackHoleAudioCaptureModule::SetStereoRecording(bool enable)
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::StereoRecording(bool* enabled) const
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::PlayoutDelay(uint16_t* delayMS) const
{
    *delayMS = 0;
    return 0;
}

bool BlackHoleAudioCaptureModule::BuiltInAECIsAvailable() const
{
    return false;
}

bool BlackHoleAudioCaptureModule::BuiltInAGCIsAvailable() const
{
    return false;
}

bool BlackHoleAudioCaptureModule::BuiltInNSIsAvailable() const
{
    return false;
}

int32_t BlackHoleAudioCaptureModule::EnableBuiltInAEC(bool enable)
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::EnableBuiltInAGC(bool enable)
{
    return -1;
}

int32_t BlackHoleAudioCaptureModule::EnableBuiltInNS(bool enable)
{
    return -1;
}
