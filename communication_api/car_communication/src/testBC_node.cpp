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
    if(cars.size() > 0)
        ROS_INFO("first car in sight is %s", cars.at(0).c_str());
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


    //service clients
    ros::ServiceClient clientSM = nh.serviceClient<car_communication::sendMessageService>("sendMessageService");
    ros::ServiceClient clientBC = nh.serviceClient<car_communication::broadcastMessageService>("broadcastMessageService");


    ros::Rate loop_rate(loop);

    // sendMessage stuff
    adhoc_customize::CarMsg testMsg;
    testMsg.dst_car = dest_car;
    car_communication::sendMessageService sMS;

    //broadcastMessage stuff
    adhoc_customize::CarMsg bcMsg;
    bcMsg.dst_car = "all";
    car_communication::broadcastMessageService bcMS;

    int counter=0;

    while(ros::ok()){

        if(counter == 0)
            ROS_INFO("loop for broadcastMessageService started");


        bcMS.request.msg = bcMsg;
        if(clientBC.call(bcMS)){
            ROS_INFO("broadcast nr. %d successful", counter+1);
        }else{
            ROS_ERROR("broadcast nr. %d failed",counter+1);
        }



        if(counter == 20)
            break;

        ++counter;

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
    return 0;

}

