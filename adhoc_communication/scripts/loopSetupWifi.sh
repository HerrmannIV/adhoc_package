#!/bin/bash
while true; do
	re=$(iwconfig wlan0 | grep cars);
	if [[ $re == *"cars"* ]]
		then
			echo "alles jut";
		else
			echo "set again";
			./setupWifi.sh;
	fi
sleep 1s;
done
