#! /bin/bash

sleep 3
sudo systemctl mask wpa_supplicant
sudo systemctl stop wpa_supplicant
sudo ip addr flush dev wlan0
sleep 0.5
sudo ifconfig wlan0 down
sleep 1
sudo ifconfig wlan0 up

sudo hostapd -B /etc/hostapd.conf
sudo ifconfig wlan0 192.168.8.88 netmask 255.255.255.0
sudo systemctl start isc-dhcp-server
sudo systemctl enable isc-dhcp-server
