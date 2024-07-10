#include <OpenteraWebrtcNativeClient/Sinks/EncodedVideoSink.h>

#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(EncodedVideoSinkTests, VideoCodecType_shouldMatchWebrtcVideoCodecType)
{
    EXPECT_EQ(VideoCodecType::Generic, static_cast<VideoCodecType>(webrtc::kVideoCodecGeneric));
    EXPECT_EQ(VideoCodecType::VP8, static_cast<VideoCodecType>(webrtc::kVideoCodecVP8));
    EXPECT_EQ(VideoCodecType::VP9, static_cast<VideoCodecType>(webrtc::kVideoCodecVP9));
    EXPECT_EQ(VideoCodecType::AV1, static_cast<VideoCodecType>(webrtc::kVideoCodecAV1));
    EXPECT_EQ(VideoCodecType::H264, static_cast<VideoCodecType>(webrtc::kVideoCodecH264));
}
