#include <OpenteraWebrtcNativeClientPython/Configurations/AudioSourceConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/DataChannelConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/SignalingServerConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/VideoSourceConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/VideoStreamConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/WebrtcConfigurationPython.h>

#include <OpenteraWebrtcNativeClientPython/Utils/ClientPython.h>
#include <OpenteraWebrtcNativeClientPython/Utils/IceServerPython.h>

#include <OpenteraWebrtcNativeClientPython/Sources/AudioSourcePython.h>
#include <OpenteraWebrtcNativeClientPython/Sources/VideoSourcePython.h>

#include <OpenteraWebrtcNativeClientPython/DataChannelClientPython.h>
#include <OpenteraWebrtcNativeClientPython/WebrtcClientPython.h>
#include <OpenteraWebrtcNativeClientPython/StreamClientPython.h>

using namespace opentera;
namespace py = pybind11;

PYBIND11_MODULE(_opentera_webrtc_native_client, m)
{
    initAudioSourceConfigurationPython(m);
    initDataChannelConfigurationPython(m);
    initSignalingServerConfigurationPython(m);
    initVideoSourceConfigurationPython(m);
    initVideoStreamConfigurationPython(m);
    initWebrtcConfigurationPython(m);

    initClientPython(m);
    initIceServerPython(m);

    initAudioSourcePython(m);
    initVideoSourcePython(m);

    initWebrtcClientPython(m);
    initDataChannelClientPython(m);
    initStreamClientPython(m);

#ifdef OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION
    m.attr("__version__") = OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION;
#else
    m.attr("__version__") = "dev";
#endif
}
