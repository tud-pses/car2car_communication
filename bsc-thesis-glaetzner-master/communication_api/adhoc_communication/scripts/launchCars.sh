#!/bin/bash
sudo ifconfig wlp2s0 down 
 sudo iwconfig wlp2s0 mode ad-hoc
 sudo iwconfig wlp2s0 essid cars
 sudo iwconfig wlp2s0 ap fe:ed:de:ad:be:ef
sudo ifconfig wlp2s0 up
 rosrun adhoc_communication adhoc_communication_node
