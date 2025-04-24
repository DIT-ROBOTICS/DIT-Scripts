#!/bin/bash

DIT_HOME="/home/ditrobotics"

# Check if the current user is root
if [ "$(id -u)" != "0" ]; then
   echo -e "This script must be run as root. \nPlease run again with 'sudo $0'"
   exit 1
fi

# Restore firefox configuration
restore_firefox() {
   echo -e "\033[32mRestoring firefox user preference...\033[0m"
   # You need to open firefox first to create the folder
   read -p "Please open firefox first. Press [Enter] key to continue..."
   if [ -d "$DIT_HOME/snap/firefox/common/.mozilla/firefox/" ]; then
      rm -rf "$DIT_HOME/snap/firefox/common/.mozilla/firefox/"
   fi
   mkdir -p "$DIT_HOME/snap/firefox/common/.mozilla/firefox/"
   tar -xzf "$DIT_HOME/DIT-Scripts/.config/firefox/firefox_profile_backup.tar.gz" -C "$DIT_HOME/snap/firefox/common/.mozilla/firefox/"
   chown -R ditrobotics:ditrobotics "$DIT_HOME/snap/firefox/common/.mozilla/firefox/"
   chmod -R 700 "$DIT_HOME/snap/firefox/common/.mozilla/firefox/"
}

# Restore desktop configuration
restore_desktop() {
   echo -e "\033[32mRestoring desktop user preference...\033[0m"
   
   cp -r "$DIT_HOME/DIT-Scripts/desktop/"* "$DIT_HOME/Desktop/"
   
   if [ ! -d /home/share/scripts/ ]; then
      mkdir -p /home/share/scripts/
   fi
   cp -r "$DIT_HOME/DIT-Scripts/share/scripts/"* /home/share/scripts/
   
   if [ ! -d /home/share/data/ ]; then
      mkdir -p /home/share/data/
   fi
   cp -r "$DIT_HOME/DIT-Scripts/share/data/"* /home/share/data/
}

# Restore plymouth theme configuration
restore_plymouth() {
   echo -e "\033[32mRestoring plymouth theme preference...\033[0m"
   cp -r "$DIT_HOME/DIT-Scripts/system/plymouth-themes/abstract_ring_alt" /usr/share/plymouth/themes/
   update-alternatives --install /usr/share/plymouth/themes/default.plymouth default.plymouth /usr/share/plymouth/themes/abstract_ring_alt/abstract_ring_alt.plymouth 100
   echo -e "\033[32mSelect the number for installed theme...\033[0m"
   update-alternatives --config default.plymouth
   update-initramfs -u
}

if [ "$1" == "all" ]; then
   restore_firefox
   restore_desktop
   restore_plymouth
else
   case "$1" in
      firefox)
         restore_firefox
         ;;
      desktop)
         restore_desktop
         ;;
      plymouth)
         restore_plymouth
         ;;
      *)
         echo "Usage: $0 {all|firefox|desktop|plymouth}"
         exit 1
         ;;
   esac
fi
