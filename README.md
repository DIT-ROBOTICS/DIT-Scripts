
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