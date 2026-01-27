#!/bin/bash
sudo cp /etc/ntpsec/ntp.conf.bak /etc/ntpsec/ntp.conf
sudo systemctl stop ntp
sudo systemctl daemon-reload
sudo systemctl start ntp
