#!/usr/bin/env sh
# Remove opentera_webrtc.native_client._opentera_webrtc_native_client as a prefix for fully-qualified names
find . -type f -name '*.html' | xargs sed -i.bak 's/opentera_webrtc\.native_client\._opentera_webrtc_native_client\.//g' && find . -type f -name '*.html.bak' | xargs rm

# Remove _opentera_webrtc_native_client
find . -type f -name '*.html' | xargs sed -i.bak 's/opentera_webrtc\.native_client\._opentera_webrtc_native_client/opentera_webrtc.native_client/g'  && find . -type f -name '*.html.bak' | xargs rm

# Fix opentera:: prefix
find . -type f -name '*.html' | xargs sed -i.bak 's/opentera:://g'  && find . -type f -name '*.html.bak' | xargs rm
