#!/bin/bash

SCRIPT=`realpath $0`
SCRIPT_PATH=`dirname $SCRIPT`

cd $SCRIPT_PATH/../../signaling-server
python3 opentera-signaling-server --port 8080 --password abc --ice_servers $SCRIPT_PATH/iceServers.json
