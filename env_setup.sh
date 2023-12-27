#!/bin/bash

# Check if the current user is root
if [ "$(id -u)" != "0" ]; then
   echo -e "This script must be run as root. \nPlease run again with 'sudo $0'"
   exit 1
fi


# Define the source files for symlinks
src_dit_logger="/home/ditrobotics/DIT-S/DIT-Logger"
src_login_launch="/home/ditrobotics/DIT-S/Login-Launch"

# The current user's username
current_user="ditrobotics"

src_DIT="/var/log/DIT"
mkdir $src_DIT
touch "$src_DIT/$HOSTNAME.log"
chgrp sudo "$src_DIT/$HOSTNAME.log"
chgrp sudo $src_DIT
chmod 750 $src_DIT
chmod 660 "$src_DIT/$HOSTNAME.log"

# Loop through each user in the sudo group
for user in $(grep '^sudo:' /etc/group | cut -d: -f4 | tr ',' ' '); do
    # Exclude the current user
    if [[ "$user" != "$current_user" ]]; then
        # Get the home directory of the user
        user_home=$(getent passwd "$user" | cut -d: -f6)

        # Create a 'Scripts' directory in the user's home directory
        mkdir -p "$user_home/Scripts"

        # Function to create symlink if the target does not exist
        create_symlink() {
            local src=$1
            local dest=$2
            local link_name=$(basename "$src")
            if [[ ! -e "$dest/$link_name" ]]; then
                ln "$src" "$dest"
            else
                echo "Symlink $dest/$link_name already exists, skipping."
            fi
        }

        # Create symlinks for DIT-Logger and Login-Launch in the user's 'Scripts' directory
        create_symlink "$src_dit_logger" "$user_home/Scripts"
        create_symlink "$src_login_launch" "$user_home/Scripts"

        # Change the owner of the Scripts directory to the user
        chown "$user":"$user" "$user_home/Scripts"

	# Create user log files
	touch "/var/log/DIT/last_login_$user"
	chown "$user":"$user" "/var/log/DIT/last_login_$user"

	# Create bash aliases
	if [ ! -f "$user_home/.bash_aliases" ]; then
	    echo "./Scripts/Login-Launch" >> "$user_home/.bash_aliases"
        else
            echo "File already exists: .bash_aliases"
        fi
    fi
done

