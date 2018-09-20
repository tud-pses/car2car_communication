#include "ros/ros.h"

#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"

#include "car_communication/sendMessageService.h"
#include "car_communication/broadcastMessageService.h"

#include <list>
#include <stdlib.h>
#include <string>
#include <vector>


void msgCallback(const adhoc_customize::CarMsg::ConstPtr &receivedMsg){
    ROS_INFO("USER_NODE received message %f", receivedMsg->msg_ID);

}


void carsInSightCallback(const adhoc_customize::CarsInSight::ConstPtr &receivedMsg){
    std::vector<std::string> cars = receivedMsg->cars;
    std::string output = "cars in sight : ";

    for (int i =0 ; i<cars.size(); ++i){
        output.append(cars.at(i) + "   |   ");
    }
    ROS_INFO("%s", output.c_str());
}

int main(int argc, char **argv){

    ros::init(argc, argv, "user_node");
    ros::NodeHandle nh;

    std::string dest_car;
    nh.getParam("/user_node/dest_car", dest_car);

    int loop = 10;
    nh.getParam("/user_node/loop", loop);


    //subscribers
    ros::Subscriber msgSub = nh.subscribe("/msgs_to_user", 10, msgCallback);
    ros::Subscriber cisSub =nh.subscribe("/cars_in_sight", 10, carsInSightCallback);


    ros::spin();
    return 0;

}

