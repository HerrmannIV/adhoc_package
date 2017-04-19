#!/bin/bash
while true; do
	re=$(iwconfig wlan0);
	if [[ $re == *"cars"* ]]
		then
		if [[ $re == *"FE:ED:DE:AD:BE:EF"* ]]
			then
				echo "ok";
			else
				echo "set again";
				sudo ifconfig wlan0 down;
				sudo iwconfig wlan0 mode ad-hoc;
				sudo iwconfig wlan0 essid cars;
				sudo iwconfig wlan0 ap fe:ed:de:ad:be:ef;
				sudo iwconfig wlan0 txpower 15;
				sudo ifconfig wlan0 up;
		fi
		else
			echo "set again";
			sudo ifconfig wlan0 down;
			sudo iwconfig wlan0 mode ad-hoc;
			sudo iwconfig wlan0 essid cars;
			sudo iwconfig wlan0 ap fe:ed:de:ad:be:ef;
			sudo iwconfig wlan0 txpower 15;
			sudo ifconfig wlan0 up;
	fi
sleep 1s;
done
