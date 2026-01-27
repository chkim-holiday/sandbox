#!/bin/bash
IFACE=${1}  
# if not provided, show error message and exit

if [ -z "$1" ]; then
    echo "Error: Need interface name as argument."
    echo "Usage: $0 <interface name>"
    exit 1
fi

echo "Network Interface: $IFACE"
echo "Local PTP Master Mode"
sudo ptp4l -i "$IFACE" -m -S
