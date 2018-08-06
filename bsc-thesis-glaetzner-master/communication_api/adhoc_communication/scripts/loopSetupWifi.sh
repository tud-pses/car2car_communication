#!/bin/bash
while true; do
	re=$(iwconfig wlp2s0);
	if [[ $re == *"cars"* ]]
		then
		if [[ $re == *"FE:ED:DE:AD:BE:EF"* ]]
			then
				echo "ok";
			else
				echo "set again";
				sudo ifconfig wlp2s0 down;
				sudo iwconfig wlp2s0 mode ad-hoc;
				sudo iwconfig wlp2s0 essid cars;
				sudo iwconfig wlp2s0 ap fe:ed:de:ad:be:ef;
				sudo iwconfig wlp2s0 txpower 15;
				sudo ifconfig wlp2s0 up;
		fi
		else
			echo "set again";
			sudo ifconfig wlp2s0 down;
			sudo iwconfig wlp2s0 mode ad-hoc;
			sudo iwconfig wlp2s0 essid cars;
			sudo iwconfig wlp2s0 ap fe:ed:de:ad:be:ef;
			sudo iwconfig wlp2s0 txpower 15;
			sudo ifconfig wlp2s0 up;
	fi
sleep 1s;
done
