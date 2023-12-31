#!/bin/bash

# Check if the current user is root
if [ "$(id -u)" != "0" ]; then
   echo -e "This script must be run as root. \nPlease run again with 'sudo $0'"
   exit 1
fi

echo "Removing DIT log folder"
cp "/var/log/DIT/$HOSTNAME.log" "/var/log/$HOSTNAME.log.bak"
rm -r "/var/log/DIT"

# Loop through each user in the sudo group
for user in $(grep '^sudo:' /etc/group | cut -d: -f4 | tr ',' ' '); do
    # Exclude the current user

    if [[ "$user" != "ditrobotics" ]]; then
        # Get the home directory of the user
        user_home=$(getent passwd "$user" | cut -d: -f6)
        
	echo "Removing $user_home/Scripts"
	rm -r "$user_home/Scripts"
	
    fi
done

