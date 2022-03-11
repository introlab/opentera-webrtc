#!/bin/bash

SCRIPT=`realpath $0`
SCRIPT_PATH=`dirname $SCRIPT`

cd $SCRIPT_PATH/..
npm install
npm run build:umd

cp $SCRIPT_PATH/../dist/openteraWebrtcWebClient.js* $SCRIPT_PATH

cd $SCRIPT_PATH/../../signaling-server
python3 opentera-signaling-server --port 8080 --password abc --ice_servers $SCRIPT_PATH/iceServers.json --static_folder $SCRIPT_PATH &
SERVER_PID=$!
trap "kill ${SERVER_PID}; exit 1" INT

URL="http://localhost:8080/tests.html"


if which xdg-open > /dev/null
then
  xdg-open $URL
elif which gnome-open > /dev/null
then
  gnome-open $URL
elif open > /dev/null
then
  open $URL
else
  echo "The OS is not supported"
fi

wait $SERVER_PID
