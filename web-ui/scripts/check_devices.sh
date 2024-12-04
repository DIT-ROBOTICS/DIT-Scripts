#!/bin/bash

OUTPUT_JSON="/home/share/data/usb.json"

# Initialize the JSON file to store device status
echo '{"stm_00":0, "stm_01":0, "lidar":0, "esp32":0, "vive_tracker":0}' > $OUTPUT_JSON

# Function to check presence of devices
function check_devices {
    for device in stm_00 stm_01 lidar esp32 vive_tracker
    do
        # Check if the device file exists in /dev
        if [ -e "/dev/$device" ]; then
            # If the device exists, update the corresponding value in the JSON file to 1
            jq ".$device = 1" $OUTPUT_JSON > tmp.$$.json && mv tmp.$$.json $OUTPUT_JSON
        else
            # If the device does not exist, update the corresponding value in the JSON file to 0
            jq ".$device = 0" $OUTPUT_JSON > tmp.$$.json && mv tmp.$$.json $OUTPUT_JSON
        fi
    done
}

# Main loop to regularly check the status of devices
while true
do
    check_devices
    sleep 1
done

