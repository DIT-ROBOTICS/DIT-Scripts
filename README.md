
```

    ____  __________   _____           _       __      
   / __ \/  _/_  __/  / ___/__________(_)___  / /______
  / / / // /  / /_____\__ \/ ___/ ___/ / __ \/ __/ ___/
 / /_/ // /  / /_____/__/ / /__/ /  / / /_/ / /_(__  ) 
/_____/___/ /_/     /____/\___/_/  /_/ .___/\__/____/  
                                    /_/                

```

# DIT-Scripts

This repository contains the environment deployment scripts for the DIT upper-level system, designed primarily for `Ubuntu Desktop 22.04 LTS`. It includes the installation of dependencies, deployment of the Docker environment, setup of the system status dashboard, configuration of the User Group permission framework for DIT's internal teams, and installation of `DIT-Logger`.

## Prerequisites for Running This Script

Before running this script, ensure your Ubuntu system is properly configured and the necessary programs are installed.

### A. Update System & Install Essential Programs (On Local Machine)

>Run the following commands:
>```bash
>sudo apt update
>sudo apt dist-upgrade -y
>sudo apt install openssh-server git
>sudo apt autoremove
>sudo apt autoclean
>sudo apt clean
>```

### B. Modify Wi-Fi Configuration to Resolve Connection Issues (Learn from 2024)

>1. Open the Wi-Fi configuration file
>```bash
>sudo vim /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf
>```
>2. Update the content as follows:
>```bash
>[connection]
>wifi.powersave = 2
>```
>~~wifi.powersave = 3~~
>
>Refer to the [Framework Knowledge Base](https://knowledgebase.frame.work/zh_tw/ubuntu-wi-fi-Skmizxznj) for more information.

## Usage

To install the DIT-Scripts, clone the repository to your local machine and run the `./00-pre-installation` script.

> **⚠️ Attention**: Run the script on **<ins>desktop terminal</ins>** is recommended. After the script is executed, the system will reboot and the installation will be completed.

## Issues

If you want to add new dependencies or other system changes, please open an issue.

## Other Notes

### A. How is the hostname of our robot's host named?
>The naming convention follows **DIT-[Year]-[Fixed IP Location]**.
>
>For example: If the hostname is **DIT-2025-10**, it indicates that this host belongs to **Eurobot 2025** and its IP is fixed at `192.168.X.10` (by default). Therefore, you can also address it directly using **mDNS** as `dit-2025-10.local`.

### B. How to ssh without password? (optional)
```bash
# Generate a dedicated ssh_key on your local machine  
ssh-keygen -t ed25519 -C "DIT-2025"  
# After saving the local key pair, upload the public key to remote (robot)  
# [ Friendly Reminder ] It is recommended to switch the files under the keys folder to read-only mode for the user using chmod 400 to ensure security  
cat ~/.ssh/keys/DIT-2025_ed25519.pub | ssh username@remote_host "mkdir -p ~/.ssh && touch ~/.ssh/authorized_keys && chmod -R go= ~/.ssh && cat >> ~/.ssh/authorized_keys"
```
Refer to the [DigitalOcean Tutorial](https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys-on-ubuntu-20-04) for more information.

### C. How to setup ssh_config? (recommended)
Edit the `~/.ssh/config` file on your local machine and add the following content:
```bash
Host            DIT-2025-10
Hostname        dit-2025-10.local
User            main
ForwardAgent    yes
IdentityFile	~/.ssh/keys/DIT-2025_ed25519
```

### D. How ssh forwarding works? (**Highly Recommended**)
Forwarding your personal ssh key to the remote host can help you access GitHub repos without leaking your personal ssh key to the robot. (**DO NOT SHARE YOUR PERSONAL SSH KEY!!!**)
```bash
ssh-add ~/.ssh/keys/[Your Personal SSH Key for GitHub]
```
