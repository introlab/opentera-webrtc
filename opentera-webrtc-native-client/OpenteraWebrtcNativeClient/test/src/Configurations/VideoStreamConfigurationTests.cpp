#include <OpenteraWebrtcNativeClient/Configurations/VideoStreamConfiguration.h>
#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(VideoStreamConfigurationTests, create_shouldSetTheAttributes)
{
    VideoStreamConfiguration testee = VideoStreamConfiguration::create();

    EXPECT_EQ(testee.forcedCodecs(), std::unordered_set<VideoStreamCodec>({}));
    EXPECT_EQ(testee.forceGStreamerHardwareAcceleration(), false);
    EXPECT_EQ(testee.useGStreamerSoftwareEncoderDecoder(), false);
}

TEST(VideoStreamConfigurationTests, create_forcedCodecs_shouldSetTheAttributes)
{
    VideoStreamConfiguration testee = VideoStreamConfiguration::create({VideoStreamCodec::VP8});

    EXPECT_EQ(testee.forcedCodecs(), std::unordered_set<VideoStreamCodec>({VideoStreamCodec::VP8}));
    EXPECT_EQ(testee.forceGStreamerHardwareAcceleration(), false);
    EXPECT_EQ(testee.useGStreamerSoftwareEncoderDecoder(), false);
}

TEST(VideoStreamConfigurationTests, create_all_shouldSetTheAttributes)
{
    VideoStreamConfiguration testee1 = VideoStreamConfiguration::create({VideoStreamCodec::VP9}, false, true);

    EXPECT_EQ(testee1.forcedCodecs(), std::unordered_set<VideoStreamCodec>({VideoStreamCodec::VP9}));
    EXPECT_EQ(testee1.forceGStreamerHardwareAcceleration(), false);
    EXPECT_EQ(testee1.useGStreamerSoftwareEncoderDecoder(), true);


    VideoStreamConfiguration testee2 = VideoStreamConfiguration::create({VideoStreamCodec::VP8, VideoStreamCodec::H264}, true, false);

    EXPECT_EQ(testee2.forcedCodecs(), std::unordered_set<VideoStreamCodec>({VideoStreamCodec::VP8, VideoStreamCodec::H264}));
    EXPECT_EQ(testee2.forceGStreamerHardwareAcceleration(), true);
    EXPECT_EQ(testee2.useGStreamerSoftwareEncoderDecoder(), false);
}
