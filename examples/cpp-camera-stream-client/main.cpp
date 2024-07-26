#include "AlsaPcmDevice.h"

#include <OpenteraWebrtcNativeClient/StreamClient.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <alsa/asoundlib.h>

#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <cstdlib>

using namespace opentera;
using namespace std;

class CvCameraCaptureVideoSource : public VideoSource
{
    atomic_bool m_stopped;
    thread m_thread;

public:
    CvCameraCaptureVideoSource()
        : VideoSource(VideoSourceConfiguration::create(false, true)),
          m_stopped(false),
          m_thread(&CvCameraCaptureVideoSource::run, this)
    {
    }

    ~CvCameraCaptureVideoSource() override
    {
        m_stopped.store(true);
        m_thread.join();
    }

private:
    void run()
    {
        constexpr int CameraIndex = 0;

        cv::VideoCapture cap;
        cap.open(CameraIndex);
        if (!cap.isOpened())
        {
            cerr << "No webcam found" << endl;
            exit(EXIT_FAILURE);
        }

        cv::Mat bgrImg;
        while (!m_stopped.load())
        {
            cap.read(bgrImg);
            if (!bgrImg.empty())
            {
                int64_t timestampUs =
                    chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now().time_since_epoch()).count();
                sendFrame(bgrImg, timestampUs);
            }
        }
    }
};

constexpr uint32_t SoundCardTotalDelayMs = 0;
constexpr bool EchoCancellation = true;
constexpr bool AutoGainControl = true;
constexpr bool NoiseSuppression = true;
constexpr bool HighpassFilter = false;
constexpr bool StereoSwapping = false;
constexpr bool TransientSuppression = false;

constexpr int BitsPerSample = 16;
constexpr PcmAudioFrameFormat AudioFormat = PcmAudioFrameFormat::Signed16;
constexpr int SampleRate = 48000;
constexpr size_t FrameSampleCount = SampleRate / 100;  // 10 ms
constexpr size_t NumberOfChannels = 1;

class AlsaAudioSource : public AudioSource
{
    atomic_bool m_stopped;
    thread m_thread;

public:
    AlsaAudioSource()
        : AudioSource(
              AudioSourceConfiguration::create(
                  SoundCardTotalDelayMs,
                  EchoCancellation,
                  AutoGainControl,
                  NoiseSuppression,
                  HighpassFilter,
                  StereoSwapping,
                  TransientSuppression),
              BitsPerSample,
              SampleRate,
              NumberOfChannels),
          m_stopped(false),
          m_thread(&AlsaAudioSource::run, this)
    {
    }

    ~AlsaAudioSource() override
    {
        m_stopped.store(true);
        m_thread.join();
    }

private:
    void run()
    {
        AlsaPcmDevice captureDevice(
            "default",
            AlsaPcmDevice::Stream::Capture,
            AudioFormat,
            NumberOfChannels,
            FrameSampleCount,
            SampleRate);

        PcmAudioFrame frame(AudioFormat, NumberOfChannels, FrameSampleCount);
        while (!m_stopped.load())
        {
            captureDevice.read(frame);
            sendFrame(frame.data(), frame.size() / bytesPerFrame());

            captureDevice.wait();
        }
    }
};

int main(int argc, char* argv[])
{
    vector<IceServer> iceServers;
    if (!IceServer::fetchFromServer("http://localhost:8080/iceservers", "abc", iceServers))
    {
        cout << "IceServer::fetchFromServer failed" << endl;
        iceServers.clear();
    }

    AlsaPcmDevice playbackDevice(
        "default",
        AlsaPcmDevice::Stream::Playback,
        AudioFormat,
        NumberOfChannels,
        FrameSampleCount,
        SampleRate);

    auto signalingServerConfiguration =
        SignalingServerConfiguration::create("ws://localhost:8080/signaling", "C++", "chat", "abc");
    auto webrtcConfiguration = WebrtcConfiguration::create(iceServers);
    auto videoStreamConfiguration = VideoStreamConfiguration::create();
    auto videoSource = make_shared<CvCameraCaptureVideoSource>();
    auto audioSource = make_shared<AlsaAudioSource>();
    StreamClient
        client(signalingServerConfiguration, webrtcConfiguration, videoStreamConfiguration, videoSource, audioSource);

    client.setOnSignalingConnectionOpened(
        []()
        {
            // This callback is called from the internal client thread.
            cout << "OnSignalingConnectionOpened" << endl;
        });
    client.setOnSignalingConnectionClosed(
        []()
        {
            // This callback is called from the internal client thread.
            cout << "OnSignalingConnectionClosed" << endl;
        });
    client.setOnSignalingConnectionError(
        [](const string& error)
        {
            // This callback is called from the internal client thread.
            cout << "OnSignalingConnectionError:" << endl << "\t" << error;
        });

    client.setOnRoomClientsChanged(
        [](const vector<RoomClient>& roomClients)
        {
            // This callback is called from the internal client thread.
            cout << "OnRoomClientsChanged:" << endl;
            for (const auto& c : roomClients)
            {
                cout << "\tid=" << c.id() << ", name=" << c.name() << ", isConnected=" << c.isConnected() << endl;
            }
        });

    client.setOnClientConnected(
        [](const Client& client)
        {
            // This callback is called from the internal client thread.
            cout << "OnClientConnected:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
            cv::namedWindow(client.id(), cv::WINDOW_AUTOSIZE);
        });
    client.setOnClientDisconnected(
        [](const Client& client)
        {
            // This callback is called from the internal client thread.
            cout << "OnClientDisconnected:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
            cv::destroyWindow(client.id());
        });
    client.setOnClientConnectionFailed(
        [](const Client& client)
        {
            // This callback is called from the internal client thread.
            cout << "OnClientConnectionFailed:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        });

    client.setOnError(
        [](const string& error)
        {
            // This callback is called from the internal client thread.
            cout << "error:" << endl;
            cout << "\t" << error << endl;
        });

    client.setLogger(
        [](const string& message)
        {
            // This callback is called from the internal client thread.
            cout << "log:" << endl;
            cout << "\t" << message << endl;
        });

    client.setOnAddRemoteStream(
        [](const Client& client)
        {
            // This callback is called from the internal client thread.
            cout << "OnAddRemoteStream:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        });
    client.setOnRemoveRemoteStream(
        [](const Client& client)
        {
            // This callback is called from the internal client thread.
            cout << "OnRemoveRemoteStream:" << endl;
            cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        });
    client.setOnVideoFrameReceived(
        [](const Client& client, const cv::Mat& bgrImg, uint64_t timestampUs)
        {
            // This callback is called from a WebRTC processing thread.
            cv::imshow(client.id(), bgrImg);
            cv::waitKey(1);
        });
    client.setOnMixedAudioFrameReceived(
        [&](const void* audioData, int bitsPerSample, int sampleRate, size_t numberOfChannels, size_t numberOfFrames)
        {
            if (bitsPerSample == BitsPerSample && numberOfChannels == NumberOfChannels && sampleRate == SampleRate)
            {
                playbackDevice.write(PcmAudioFrame(AudioFormat, NumberOfChannels, numberOfFrames, (uint8_t*)audioData));
            }
            else
            {
                cout << "Receiving not supported audio frame (bitsPerSample=" << bitsPerSample
                     << ", numberOfChannels=" << numberOfChannels << ", sampleRate=" << sampleRate << ")" << endl;
            }
        });

    client.connect();

    cin.get();

    return 0;
}
