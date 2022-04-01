#include <OpenteraWebrtcNativeClient/VideoEncoder/ForceH264VideoEncoderFactory.h>

#include <api/video_codecs/video_encoder.h>
#include <api/video_codecs/sdp_video_format.h>

#include <algorithm>

using namespace opentera;
using namespace std;

ForceH264VideoEncoderFactory::ForceH264VideoEncoderFactory(unique_ptr<webrtc::VideoEncoderFactory> videoEncoderFactory)
    : m_videoEncoderFactory(move(videoEncoderFactory))
{
}

ForceH264VideoEncoderFactory::~ForceH264VideoEncoderFactory() {}

vector<webrtc::SdpVideoFormat> ForceH264VideoEncoderFactory::GetSupportedFormats() const
{
    return filterSdpFormats(m_videoEncoderFactory->GetSupportedFormats());
}

ForceH264VideoEncoderFactory::CodecInfo
    ForceH264VideoEncoderFactory::QueryVideoEncoder(const webrtc::SdpVideoFormat& format) const
{
    return m_videoEncoderFactory->QueryVideoEncoder(format);
}

unique_ptr<webrtc::VideoEncoder> ForceH264VideoEncoderFactory::CreateVideoEncoder(const webrtc::SdpVideoFormat& format)
{
    return m_videoEncoderFactory->CreateVideoEncoder(format);
}

#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_JETSON
vector<webrtc::SdpVideoFormat> ForceH264VideoEncoderFactory::GetImplementations() const
{
    return filterSdpFormats(m_videoEncoderFactory->GetImplementations());
}

unique_ptr<ForceH264VideoEncoderFactory::EncoderSelectorInterface>
    ForceH264VideoEncoderFactory::GetEncoderSelector() const
{
    return m_videoEncoderFactory->GetEncoderSelector();
}
#endif

vector<webrtc::SdpVideoFormat>
    ForceH264VideoEncoderFactory::filterSdpFormats(const vector<webrtc::SdpVideoFormat>& formats)
{
    vector<webrtc::SdpVideoFormat> filteredFormats;
    copy_if(
        formats.begin(),
        formats.end(),
        back_inserter(filteredFormats),
        [](const webrtc::SdpVideoFormat& format) { return format.name == "H264"; });

    return filteredFormats;
}

unique_ptr<webrtc::VideoEncoderFactory>
    opentera::createForceH264VideoEncoderFactory(unique_ptr<webrtc::VideoEncoderFactory> videoEncoderFactory)
{
    return make_unique<ForceH264VideoEncoderFactory>(move(videoEncoderFactory));
}
