#!/bin/bash

# Check if the current user is root
if [ "$(id -u)" != "0" ]; then
   echo -e "This script must be run as root. \nPlease run again with 'sudo $0'"
   exit 1
fi

usernames=("main" "vision" "navigation" "localization")

for username in "${usernames[@]}"
do
    echo "Creating user: $username"
    adduser $username
    echo "$username created successfully!"

    echo "Adding $username to the sudo group"
    usermod -aG sudo $username

    echo "Adding $username to the docker group"
    usermod -aG docker $username

    echo "$username has been added to the sudo and docker groups successfully!"
done

echo "All users have been created and added to the necessary groups successfully!"

