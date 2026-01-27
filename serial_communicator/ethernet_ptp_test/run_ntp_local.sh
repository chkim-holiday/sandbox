#!/bin/bash
IFACE=${1:-eth0}  # 인자가 없으면 기본값 eth0
NTP_CONFIG=${2}

if [ -z "$1" ]; then
    echo "Error: Need interface name as argument."
    echo "Usage: $0 <interface name>"
    exit 1
fi
if [ -z "$2" ]; then
    echo "Error: Need NTP config file as argument."
    echo "Usage: $0 <interface name> <ntp config file>"
    exit 1
fi

# If ntp.conf exist,
if [ -f /etc/ntpsec/ntp.conf ]; then
    echo "Backing up existing NTP config."
    sudo cp /etc/ntpsec/ntp.conf /etc/ntpsec/ntp.conf.bak
fi
sudo ln -sf "$NTP_CONFIG" /etc/ntpsec/ntp.conf
sudo systemctl stop ntp
sudo systemctl daemon-reload
sudo systemctl start ntp

echo "NTP configuration applied from $NTP_CONFIG"

sudo netstat -ulnp | grep ntpd