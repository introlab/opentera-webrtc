#include <OpenteraWebrtcNativeClientPython/Configurations/AudioSourceConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/DataChannelConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/SignallingServerConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/VideoSourceConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/WebrtcConfigurationPython.h>

#include <OpenteraWebrtcNativeClientPython/Utils/ClientPython.h>
#include <OpenteraWebrtcNativeClientPython/Utils/IceServerPython.h>

#include <OpenteraWebrtcNativeClientPython/Sources/AudioSourcePython.h>
#include <OpenteraWebrtcNativeClientPython/Sources/VideoSourcePython.h>

#include <OpenteraWebrtcNativeClientPython/SignallingClientPython.h>
#include <OpenteraWebrtcNativeClientPython/DataChannelClientPython.h>
#include <OpenteraWebrtcNativeClientPython/StreamClientPython.h>

using namespace opentera;
namespace py = pybind11;

PYBIND11_MODULE(opentera_webrtc_native_client, m)
{
    initAudioSourceConfigurationPython(m);
    initDataChannelConfigurationPython(m);
    initSignallingServerConfigurationPython(m);
    initVideoSourceConfigurationPython(m);
    initWebrtcConfigurationPython(m);

    initClientPython(m);
    initIceServerPython(m);

    initAudioSourcePython(m);
    initVideoSourcePython(m);

    initSignallingClientPython(m);
    initDataChannelClientPython(m);
    initStreamClientPython(m);
}
