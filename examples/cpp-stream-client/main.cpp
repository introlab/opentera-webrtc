#include <OpenteraWebrtcNativeClient/StreamClient.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

using namespace opentera;
using namespace std;

constexpr chrono::milliseconds NoiseVideoSourceFrameDuration = 100ms;

class NoiseVideoSource : public VideoSource
{
    atomic_bool m_stopped;
    thread m_thread;

public:
    NoiseVideoSource() : VideoSource(VideoSourceConfiguration::create(false, true)), m_stopped(false),
            m_thread(&NoiseVideoSource::run, this)
    {
    }

    ~NoiseVideoSource() override
    {
        m_stopped.store(true);
        m_thread.join();
    }

private:
    void run()
    {
        cv::Mat bgrImg(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
        while(!m_stopped.load())
        {
            cv::randu(bgrImg, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
            this_thread::sleep_for(NoiseVideoSourceFrameDuration);
            int64_t timestampUs =
                    chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now().time_since_epoch()).count();
            sendFrame(bgrImg, timestampUs);
        }
    }
};

constexpr int BitsPerSample = 16;
constexpr int SampleRate = 48000;
constexpr size_t NumberOfChannels = 1;
constexpr chrono::milliseconds SinAudioSourceFrameDuration = 10ms;
constexpr chrono::milliseconds SinAudioSourceSleepBuffer = 2ms;
constexpr int16_t SinAudioSourceAmplitude = 15000;

class SinAudioSource : public AudioSource
{
    atomic_bool m_stopped;
    thread m_thread;

public:
    SinAudioSource() : AudioSource(AudioSourceConfiguration::create(), BitsPerSample, SampleRate, NumberOfChannels),
            m_stopped(false),
            m_thread(&SinAudioSource::run, this)
    {
    }

    ~SinAudioSource() override
    {
        m_stopped.store(true);
        m_thread.join();
    }

private:
    void run()
    {
        vector<int16_t> data(SinAudioSourceFrameDuration.count() * SampleRate / 1000, 0);
        double t = 0;
        for (size_t i = 0; i < data.size(); i++)
        {
            data[i] = static_cast<int16_t>(SinAudioSourceAmplitude * sin(t));
            t += 2 * M_PI / data.size();
        }

        while(!m_stopped.load())
        {
            sendFrame(data.data(), data.size() * sizeof(int16_t) / bytesPerFrame());

            auto start = chrono::steady_clock::now();
            this_thread::sleep_for(SinAudioSourceFrameDuration - SinAudioSourceSleepBuffer);
            while ((chrono::steady_clock::now() - start) < SinAudioSourceFrameDuration);
        }
    }
};

int main(int argc, char* argv[])
{
    auto signalingServerConfiguration =
            SignalingServerConfiguration::create("http://localhost:8080", "C++", "chat", "abc");
    auto webrtcConfiguration = WebrtcConfiguration::create();
    auto videoSource = make_shared<NoiseVideoSource>();
    auto audioSource = make_shared<SinAudioSource>();
    StreamClient client(signalingServerConfiguration, webrtcConfiguration, videoSource, audioSource);

    client.setOnSignalingConnectionOpen([]()
    {
        cout << "OnSignalingConnectionOpen" << endl;
    });
    client.setOnSignalingConnectionClosed([]()
    {
        cout << "OnSignalingConnectionClosed" << endl;
    });
    client.setOnSignalingConnectionError([](const string& error)
    {
        cout << "OnSignalingConnectionClosed:" << endl << "\t" << error;
    });

    client.setOnRoomClientsChanged([](const vector<RoomClient>& roomClients)
    {
        cout << "OnRoomClientsChanged:" << endl;
        for (const auto& c : roomClients)
        {
            cout << "\tid=" << c.id() << ", name=" << c.name() << ", isConnected=" << c.isConnected() << endl;
        }
    });

    client.setOnClientConnected([](const Client& client)
    {
        cout << "OnClientConnected:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
    });
    client.setOnClientDisconnected([](const Client& client)
    {
        cout << "OnClientDisconnected:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
    });

    client.setOnError([](const string& error)
    {
        cout << "error or warning:" << endl;
        cout << "\t" << error << endl;
    });

    client.setOnAddRemoteStream([](const Client& client)
    {
        cout << "OnDataChannelOpen:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
    });
    client.setOnRemoveRemoteStream([](const Client& client)
    {
        cout << "OnRemoveRemoteStream:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
    });
    client.setOnVideoFrameReceived([](const Client& client, const cv::Mat& bgrImg, uint64_t timestampUs)
    {
        cv::imshow(client.id(), bgrImg);
        cv::waitKey(1);
    });
    client.setOnAudioFrameReceived([](const Client& client,
        const void* audioData,
        int bitsPerSample,
        int sampleRate,
        size_t numberOfChannels,
        size_t numberOfFrames)
    {
        cout << "OnAudioFrameReceived:" << endl;
        cout << "\tid=" << client.id() << ", name=" << client.name() << endl;
        cout << "\tbitsPerSample=" << bitsPerSample << ", sampleRate=" << sampleRate;
        cout << ", numberOfChannels=" << numberOfChannels << ", numberOfFrames=" << numberOfFrames << endl;
    });

    client.connect();

    cin.get();

    return 0;
}

