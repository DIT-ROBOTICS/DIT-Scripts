#!/bin/bash

FileNAME='99-dit-news'
FilePATH='/etc/update-motd.d/'

# Check if the current user is root
if [ "$(id -u)" != "0" ]; then
   echo -e "This script must be run as root. \nPlease run again with 'sudo $0'"
   exit 1
fi

# Copy the motd message file
echo "Copying motd config to $FilePATH"
cp "$FileNAME" "$FilePATH"

