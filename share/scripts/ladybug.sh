#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 [IP Address]"
    exit 1
fi

IP_ADDRESS=$1

curl -X GET "http://$IP_ADDRESS/updates?output=1&state=1" -H "Connection: close"
