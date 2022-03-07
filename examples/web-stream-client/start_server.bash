#!/bin/bash

SCRIPT=`realpath $0`
SCRIPT_PATH=`dirname $SCRIPT`

cd $SCRIPT_PATH/../../opentera-webrtc-web-client
npm install
npm run build:umd

cp $SCRIPT_PATH/../../opentera-webrtc-web-client/dist/openteraWebrtcWebClient.js* $SCRIPT_PATH

cd $SCRIPT_PATH/../../signaling-server
python3 opentera-signaling-server --port 8080 --password abc --ice_servers $SCRIPT_PATH/iceServers.json --static_folder $SCRIPT_PATH
