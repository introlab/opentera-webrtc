#include <OpenteraWebrtcNativeClient/Configurations/VideoStreamConfiguration.h>
#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(VideoStreamConfigurationTests, stringToVideoStreamCodec)
{
    EXPECT_EQ(stringToVideoStreamCodec("VP8"), VideoStreamCodec::VP8);
    EXPECT_EQ(stringToVideoStreamCodec("VP9"), VideoStreamCodec::VP9);
    EXPECT_EQ(stringToVideoStreamCodec("H264"), VideoStreamCodec::H264);

    EXPECT_EQ(stringToVideoStreamCodec("vp8"), VideoStreamCodec::VP8);
    EXPECT_EQ(stringToVideoStreamCodec("vp9"), VideoStreamCodec::VP9);
    EXPECT_EQ(stringToVideoStreamCodec("h264"), VideoStreamCodec::H264);

    EXPECT_EQ(stringToVideoStreamCodec("Vp8"), VideoStreamCodec::VP8);
    EXPECT_EQ(stringToVideoStreamCodec("Vp9"), VideoStreamCodec::VP9);
    EXPECT_EQ(stringToVideoStreamCodec("vP8"), VideoStreamCodec::VP8);
    EXPECT_EQ(stringToVideoStreamCodec("vP9"), VideoStreamCodec::VP9);

    EXPECT_EQ(stringToVideoStreamCodec("asd"), absl::nullopt);
}

TEST(VideoStreamConfigurationTests, create_shouldSetTheAttributes)
{
    VideoStreamConfiguration testee = VideoStreamConfiguration::create();

    EXPECT_EQ(testee.forcedCodecs(), unordered_set<VideoStreamCodec>({}));
    EXPECT_EQ(testee.forceGStreamerHardwareAcceleration(), false);
    EXPECT_EQ(testee.useGStreamerSoftwareEncoderDecoder(), false);
}

TEST(VideoStreamConfigurationTests, create_forcedCodecs_shouldSetTheAttributes)
{
    VideoStreamConfiguration testee = VideoStreamConfiguration::create({VideoStreamCodec::VP8});

    EXPECT_EQ(testee.forcedCodecs(), unordered_set<VideoStreamCodec>({VideoStreamCodec::VP8}));
    EXPECT_EQ(testee.forceGStreamerHardwareAcceleration(), false);
    EXPECT_EQ(testee.useGStreamerSoftwareEncoderDecoder(), false);
}

TEST(VideoStreamConfigurationTests, create_all_shouldSetTheAttributes)
{
    VideoStreamConfiguration testee1 = VideoStreamConfiguration::create({VideoStreamCodec::VP9}, false, true);

    EXPECT_EQ(testee1.forcedCodecs(), unordered_set<VideoStreamCodec>({VideoStreamCodec::VP9}));
    EXPECT_EQ(testee1.forceGStreamerHardwareAcceleration(), false);
    EXPECT_EQ(testee1.useGStreamerSoftwareEncoderDecoder(), true);


    VideoStreamConfiguration testee2 =
        VideoStreamConfiguration::create({VideoStreamCodec::VP8, VideoStreamCodec::H264}, true, false);

    EXPECT_EQ(testee2.forcedCodecs(), unordered_set<VideoStreamCodec>({VideoStreamCodec::VP8, VideoStreamCodec::H264}));
    EXPECT_EQ(testee2.forceGStreamerHardwareAcceleration(), true);
    EXPECT_EQ(testee2.useGStreamerSoftwareEncoderDecoder(), false);
}
