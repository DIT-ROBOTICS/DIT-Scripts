#!/bin/bash

# Check if the current user is root
if [ "$(id -u)" != "0" ]; then
   echo -e "This script must be run as root. \nPlease run again with 'sudo $0'"
   exit 1
fi

# Add ROS_DOMAIN_ID to /etc/environment if not already present
if ! grep -q "ROS_DOMAIN_ID" /etc/environment; then
    ROS_DOMAIN_ID=${HOSTNAME##*-}
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> /etc/environment
    echo -e "\033[32mROS_DOMAIN_ID added to /etc/environment\033[0m"
else
    echo -e "\033[33mROS_DOMAIN_ID already exists in /etc/environment\033[0m"
fi