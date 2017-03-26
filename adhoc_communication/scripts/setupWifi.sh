#!/bin/bash
# sets the wifi parameters on wlan0 

sudo ifconfig wlan0 down
sudo iwconfig wlan0 mode ad-hoc
sudo iwconfig wlan0 essid cars
sudo iwconfig wlan0 ap fe:ed:de:ad:be:ef
sudo iwconfig wlan0 txpower 15
sudo ifconfig wlan0 up

