#!/bin/bash
while true; do
	re=$(iwconfig wlan0 | grep cars);
	if [[ $re == *"cars"* ]]
		then
			echo "alles jut";
		else
			echo "set again";
			sudo ifconfig wlan0 down;
			sudo iwconfig wlan0 mode ad-hoc;
			sudo iwconfig wlan0 essid cars;
			sudo iwconfig wlan0 ap fe:ed:de:ad:be:ef;
			sudo ifconfig wlan0 up;
	fi
sleep 1s;
done
