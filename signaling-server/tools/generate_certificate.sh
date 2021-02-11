#!/bin/bash
openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 365 -subj '/CN=localhost' -nodes
# will return the exit code of the previous command
exit
