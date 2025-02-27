#!/bin/bash

# Check if the current user is root
if [ "$(id -u)" != "0" ]; then
   echo -e "\033[31mThis script must be run as root. \nPlease run again with 'sudo $0'\033[0m"
   exit 1
fi

# Function to draw a progress bar
progress_bar() {
    # Parameters: current step, total steps
    local current_step=$1
    local total_steps=$2
    local progress=$((current_step * 100 / total_steps))
    local filled=$((progress * 50 / 100)) # Assuming 50 characters wide bar
    local empty=$((50 - filled))
    echo -ne "\r\033[32m["
    printf "%-${filled}s" '' | tr ' ' '#'
    printf "%-${empty}s" '' | tr ' ' '-'
    printf "] \033[33m%d%% (%d/%d)\033[0m \n" $progress $current_step $total_steps
    if [ $current_step -eq $total_steps ]; then
        echo "" # Print a new line at the end of the progress
    fi
}

# Initialize step and total steps variables
step=1
total_steps=10 # Adjust based on the actual number of steps you have

upgrade_system() {
    echo -e "\033[32mUpdating and upgrading system...\033[0m"
	
    apt update && sudo apt dist-upgrade -y

    sleep 1
    progress_bar $step $total_steps
    ((step++))
}

install_dependencies() {
    echo -e "\033[32mInstalling dependencies...\033[0m"

    apt install -y \
		vim \
		curl \
		net-tools \
		iw \
		lm-sensors \
		conky-all \
		tmux \
		screen \
		htop \
		tree \
		iperf3 \
		timeshift \
		cheese \
		ffmpeg \
		jq \
        ncdu \
        git-lfs

		# net-tools     --- for network configuration
		# iw            --- for wireless network configuration
		# lm-sensors    --- for device temperature monitoring
		# conky-all     --- for system monitoring panel (update conky -> conky-all for Ubuntu 24.04)
		# iperf3        --- for network performance testing
		# timeshift     --- for system snapshot and restore
		# cheese        --- for camera testing
		# ffmpeg        --- for audio and video processing (ex. boot-on sound)
		# jq            --- for JSON parsing
        # ncdu          --- for disk usage analysis
        # git-lfs       --- for git large file storage

    sleep 1
    progress_bar $step $total_steps
    ((step++))
}

# Fix for USB device detection issue
# |
# |  Reference: 
# |  https://askubuntu.com/questions/1403705/dev-ttyusb0-not-present-in-ubuntu-22-04
# |
remove_brltty() {    
    echo -e "\033[32mRemoving brltty...\033[0m"

    apt remove -y brltty

    sleep 1
    progress_bar $step $total_steps
    ((step++))
}

# Install Docker Engine
install_docker() {
    echo -e "\033[32mInstalling Docker...\033[0m"

    curl -fsSL https://get.docker.com -o get-docker.sh
    sh ./get-docker.sh
    rm get-docker.sh

    groupadd docker
    usermod -aG docker ditrobotics
    # newgrp docker # optional

    sleep 1
    progress_bar $step $total_steps
    ((step++))
}

# Setup Conky system monitoring panel
setup_conky() {
    echo -e "\033[32mSetting up conky...\033[0m"

    ./10-conky_setup.sh

    sleep 1
    progress_bar $step $total_steps
    ((step++))
}

# Setup Message of the MOTD banner
setup_motd_banner() {
    echo -e "\033[32mSetting up motd banner...\033[0m"

    ./10-motd_setup.sh

    sleep 1
    progress_bar $step $total_steps
    ((step++))
}

# Setup DIT user group
create_user() {
    echo -e "\033[32mCreate each group user and set their permission...\033[0m"

    ./10-user_setup.sh

    sleep 1
    progress_bar $step $total_steps
    ((step++))
}

# Setup DIT logger (System change information for each group)
setup_dit_logger() {
    echo -e "\033[32mSetting up DIT logger...\033[0m"

    ./env_setup.sh

    sleep 1
    progress_bar $step $total_steps
    ((step++))
}

# Setup touch screen orientation:
# |
# | This is for Eurobot 2024 external usb touch screen, it isn't needed in general.
# |
flip_screen() {
    echo -e "\033[32mSetting up touch screen HID layout...\033[0m"
	
    read -p "Would you like to rotate external touch screen? (y/N): " answer

    case $answer in
        [Yy]* )
            echo 'ATTRS{name}=="wch.cn USB2IIC_CTP_CONTROL", ENV{LIBINPUT_CALIBRATION_MATRIX}="-1.000 0.000 1.000 0.000 -1.000 1.000"' >> /etc/udev/rules.d/80-calibration.rules
            udevadm control --reload-rules
            udevadm trigger
            service udev restart
	    ;;
        * )
            echo "The screen layout will remain default."
	    ;;
    esac

    sleep 1
    progress_bar $step $total_steps
    ((step++))
}

# Restore user preference
restore_user_preference() {

    # Restore firefox configuration
    echo -e "\033[32mRestoring firefox user preference...\033[0m"
    # You need to open firefox first to create the folder
    read -p "Please open firefox first. Press [Enter] key to continue..."
    find /home/ditrobotics/snap/firefox/common/.mozilla/firefox/ -type d -name "*.default" -exec cp -r /home/ditrobotics/DIT-Scripts/.mozilla/firefox/dit_config.default/* {} \;

    # Restore desktop configuration
    echo -e "\033[32mRestoring desktop user preference...\033[0m"
    cp -r /home/ditrobotics/DIT-Scripts/desktop/* /home/ditrobotics/Desktop/

    sleep 1
    progress_bar $step $total_steps
    ((step++))
}

# Add more steps as necessary...

cd /home/ditrobotics/DIT-Scripts
echo -e "\033[32mStarting pre-installation setup process...\033[0m"
for ((i=step; i<=total_steps; i++)); do
    case $i in
        1) upgrade_system;;
        2) install_dependencies;;
        3) remove_brltty;;
        4) install_docker;;
        5) setup_conky;;
        6) setup_motd_banner;;
        7) create_user;;
        8) setup_dit_logger;;
        9) flip_screen;;
       10) restore_user_preference;;
        *) echo -e "\033[31mInvalid step\033[0m";;
    esac
done

echo -e "\033[32mSetup complete. Please review any manual actions required.\033[0m"
MOTD="

1) Please check /etc/update-motd.d/99-dit-news Wi-Fi card model
2) Please check /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf wifi.powersave = 2
3) Please check your touch screen rotated or not

REBOOT YOUR SYSTEM!!!
IF ALL LOOKS GOOD, THEN CHECK THE FOLLOWING ITEM

A) Do your first timeshift
B) [ABANDONED] Use Clonezilla backup to external SSD
C) Install Active Backup for Business (Synology NAS) and backup to NAS
"
echo "$MOTD"

