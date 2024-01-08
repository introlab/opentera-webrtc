#include <OpenteraWebrtcNativeClient/Signaling/SignalingClient.h>

using namespace opentera;
using namespace std;

SignalingClient::SignalingClient(SignalingServerConfiguration configuration) : m_configuration(move(configuration)) {}

SignalingClient::~SignalingClient() {}
