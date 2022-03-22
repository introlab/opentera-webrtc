#include <OpenteraWebrtcNativeClient/Configurations/VideoSourceConfiguration.h>

using namespace opentera;
using namespace std;

VideoSourceConfiguration::VideoSourceConfiguration(bool needsDenoising, bool isScreencast)
    : m_needsDenoising(needsDenoising),
      m_isScreencast(isScreencast)
{
}
