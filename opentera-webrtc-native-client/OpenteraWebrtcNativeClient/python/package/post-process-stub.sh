#!/usr/bin/env sh
sed -i.bak 's/_opentera_webrtc_native_client\.//g' $1 && rm $1.bak
sed -i.bak 's/opentera:://g' $1 && rm $1.bak
