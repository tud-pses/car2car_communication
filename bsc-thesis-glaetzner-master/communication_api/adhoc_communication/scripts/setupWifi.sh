#!/bin/bash
# sets the wifi parameters on wlp2s0 

sudo ifconfig wlp2s0 down
sudo iwconfig wlp2s0 mode ad-hoc
sudo iwconfig wlp2s0 essid cars
sudo iwconfig wlp2s0 ap fe:ed:de:ad:be:ef
sudo iwconfig wlp2s0 txpower 15
sudo ifconfig wlp2s0 up

