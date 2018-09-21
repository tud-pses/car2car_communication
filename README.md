# Car2Car Communication

ROS-Package containing an API for Car2Car Communication. This repository also contains a Platooning demo that makes use of the API.

### Prerequisites

This project was build with ROS Kinetic and have not been tested with other ROS versions. In addition to that this project depends on the following ROS-Package:
* [ar_track_alvar_msgs](http://wiki.ros.org/ar_track_alvar_msgs)

### Installing

Clone the repo into your ROS src folder:

`cd ~/catkin_ws/src`

`git clone https://github.com/tud-pses/car2car_communication.git`

`cd ..`

Build the package with catkin_make:

`catkin_make`

## Getting Started
1. execute the bash script `launchCars.sh` that is located in the folder `communication_api/car_communication/scripts`. This sets up the wireless network adapter in ad-hoc mode.

    `roslaunch kinect2_bridge kinect2_bridge.launch`

2. In order to start the Car2Car Communication itself, just start it via the provided launch file.

    `roslaunch car_communication car_communication_api.launch`

3. (optional) Start the Platooning demo via the provided launch file.

    `roslaunch platooning platooning.launch`
    
## Author

* **Tobias Glätzner**
