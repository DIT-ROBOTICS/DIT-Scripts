#!/bin/bash

# netstat -tn | grep :22 | awk '{print $5}' | cut -d: -f1 | sort -u
who | cut -d' ' -f2- | awk '{print $1, $2, $3, $4}' | column -t
