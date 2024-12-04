#!/bin/bash

# Check if the current user is root
if [ "$(id -u)" != "0" ]; then
   echo -e "This script must be run as root. \nPlease run again with 'sudo $0'"
   exit 1
fi

# Backing up log file
echo "Copying log file..."
cp "/var/log/DIT/$HOSTNAME.log" "/var/log/$HOSTNAME.log.bak"

# Check remove the folder or not
read -p "Do you want to delete the DIT log folder? (y/n): " answer

case $answer in
    [Yy]* )
        echo "Removing DIT log folder..."
        rm -r "/var/log/DIT"
        echo "Folder removed."
        ;;
    [Nn]* )
        echo "Folder not removed."
        ;;
    * )
        echo "Invalid response. Folder not removed."
        ;;
esac

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

