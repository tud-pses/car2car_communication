#include "ros/ros.h"

#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"

#include "car_communication/sendMessageService.h"
#include "car_communication/broadcastMessageService.h"

#include <list>
#include <stdlib.h>
#include <string>
#include <vector>


std::vector<std::string> carsInSight;

void msgCallback(const adhoc_customize::CarMsg::ConstPtr &receivedMsg){
    ROS_INFO("testSM_node received message %f", receivedMsg->msg_ID);
    return;
}


void carsInSightCallback(const adhoc_customize::CarsInSight::ConstPtr &receivedMsg){
    carsInSight = receivedMsg->cars;
    return;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "testSM");
    ros::NodeHandle nh;

    std::string dest_car;
    nh.getParam("/testSM/dest_car", dest_car);

    int loop = 10;
    nh.getParam("/testSM/loop", loop);


    //subscribers
    ros::Subscriber msgSub = nh.subscribe("/msgs_to_user", 10, msgCallback);
    ros::Subscriber cisSub =nh.subscribe("/cars_in_sight", 10, carsInSightCallback);


    //service clients
    ros::ServiceClient clientSM = nh.serviceClient<car_communication::sendMessageService>("sendMessageService");


    ros::Rate loop_rate(loop);

    // sendMessage stuff
    adhoc_customize::CarMsg testMsg;
    testMsg.dst_car = dest_car;
    car_communication::sendMessageService sMS;
    bool valid;

    int counter=0;

    while(ros::ok()){

        if(counter == 0)
            ROS_INFO("loop for sendMessageService started");

        sMS.request.msg = testMsg;

        valid = clientSM.call(sMS);
        if(valid){
            if(sMS.response.success)
                ROS_INFO("send message nr. %d succesfully to %s", counter+1, dest_car.c_str());

        }else{
            ROS_ERROR("send message nr. %d failed", counter+1);
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

