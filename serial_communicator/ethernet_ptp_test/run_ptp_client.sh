#!/bin/bash
IFACE=${1}
if [ -z "$1" ]; then
    echo "Error: Need interface name as argument."
    echo "Usage: $0 <interface name>"
    exit 1
fi

echo "Network Interface: $IFACE"
echo "Local PTP Client Mode"
sudo ptp4l -i "$IFACE" -m -S -s --step_threshold=1 --first_step_threshold=0