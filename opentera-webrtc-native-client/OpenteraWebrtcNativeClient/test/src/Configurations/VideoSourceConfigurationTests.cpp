#include <OpenteraWebrtcNativeClient/Configurations/VideoSourceConfiguration.h>

#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

TEST(VideoSourceConfigurationTests, create_shouldSetTheAttributes)
{
    VideoSourceConfiguration testee = VideoSourceConfiguration::create(true, false);

    EXPECT_EQ(testee.needsDenoising(), true);
    EXPECT_EQ(testee.isScreencast(), false);
}
