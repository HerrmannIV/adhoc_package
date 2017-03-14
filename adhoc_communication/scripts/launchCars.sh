#!/bin/bash
sudo ifconfig wlan0 down 
 sudo iwconfig wlan0 mode ad-hoc
 sudo iwconfig wlan0 essid cars
 sudo iwconfig wlan0 ap fe:ed:de:ad:be:ef
sudo ifconfig wlan0 up
 rosrun adhoc_communication adhoc_communication_node
