#!/usr/bin/env sh
# Remove opentera.webrtc.native_client._opentera_webrtc_native_client as a prefix for fully-qualified names
find . -type f -name '*.html' | xargs sed -i 's/opentera\.webrtc\.native_client\._opentera_webrtc_native_client\.//g'

# Remove _opentera_webrtc_native_client
find . -type f -name '*.html' | xargs sed -i 's/opentera\.webrtc\.native_client\._opentera_webrtc_native_client/opentera.webrtc.native_client/g'
