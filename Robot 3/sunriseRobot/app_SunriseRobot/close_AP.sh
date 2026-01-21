#! /bin/bash
sudo killall -9 hostapd

sudo ip addr flush dev wlan0
sleep 0.5
sudo ifconfig wlan0 down
sleep 1
sudo ifconfig wlan0 up

sudo systemctl unmask wpa_supplicant
sudo systemctl restart wpa_supplicant

sudo rmmod aic8800_fdrv
sudo modprobe aic8800_fdrv

