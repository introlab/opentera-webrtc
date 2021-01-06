#include <OpenteraWebrtcNativeClient/Configurations/VideoSourceConfiguration.h>

using namespace introlab;
using namespace std;

VideoSourceConfiguration::VideoSourceConfiguration(bool needsDenoising, bool isScreencast) :
        m_needsDenoising(needsDenoising), m_isScreencast(isScreencast)
{
}
