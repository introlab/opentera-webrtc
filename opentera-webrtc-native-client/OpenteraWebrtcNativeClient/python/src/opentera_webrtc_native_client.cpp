#include <OpenteraWebrtcNativeClientPython/Configurations/DataChannelConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/SignallingServerConfigurationPython.h>
#include <OpenteraWebrtcNativeClientPython/Configurations/WebrtcConfigurationPython.h>

#include <OpenteraWebrtcNativeClientPython/Utils/ClientPython.h>
#include <OpenteraWebrtcNativeClientPython/Utils/IceServerPython.h>

#include <OpenteraWebrtcNativeClientPython/SignallingClientPython.h>
#include <OpenteraWebrtcNativeClientPython/DataChannelClientPython.h>

#include <OpenteraWebrtcNativeClientPython/VideoSourcePython.h>
#include <OpenteraWebrtcNativeClientPython/StreamClientPython.h>

using namespace introlab;
namespace py = pybind11;

PYBIND11_MODULE(opentera_webrtc_native_client, m)
{
    initDataChannelConfigurationPython(m);
    initSignallingServerConfigurationPython(m);
    initWebrtcConfigurationPython(m);

    initClientPython(m);
    initIceServerPython(m);

    initSignallingClientPython(m);
    initDataChannelClientPython(m);

    initVideoSourcePython(m);
    initStreamClientPython(m);
}
