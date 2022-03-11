#!/usr/bin/env sh
sed -i 's/_opentera_webrtc_native_client\.//g' $1
sed -i 's/opentera::IceServer/IceServer/g' $1
