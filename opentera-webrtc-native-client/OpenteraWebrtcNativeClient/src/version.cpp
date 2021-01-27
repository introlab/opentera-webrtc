#include <OpenteraWebrtcNativeClient/version.h>

using namespace std;

string opentera::getVersion()
{
#ifdef OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION
    return OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION;
#else
    return "dev";
#endif
}
