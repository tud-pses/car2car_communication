#include "ros/ros.h"

#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"

#include "car_communication/sendMessageService.h"
#include "car_communication/broadcastMessageService.h"

#include <list>
#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>


std::vector<double> time_values;



void msgCallback(const adhoc_customize::CarMsg::ConstPtr &receivedMsg){

    return;
}

void printSummary(int failed){

    double current;

    //output file
    std::ofstream stream;
    stream.open("/home/pses/Documents/test_results/datarate.csv", std::ofstream::out | std::ofstream::app);

    for (int i = 0; i<time_values.size();++i){
        current = time_values.at(i);
        stream << (std::to_string(current) + ";");

    }
    stream << ("failed: " + std::to_string(failed) + ";");

    stream.close();

}



int main(int argc, char **argv){

    ros::init(argc, argv, "datarate_node");
    ros::NodeHandle nh;

    //subscribers
    ros::Subscriber msgSub = nh.subscribe("/recvMsg", 10, msgCallback);

    //service client
    ros::ServiceClient client = nh.serviceClient<car_communication::sendMessageService>("sendMessageService");

    //get params
    int size;
    nh.getParam("/datarate_node/size",size);

    int max_i_count;
    nh.getParam("/datarate_node/max_i_count", max_i_count);

    std::string dst_car;
    nh.getParam("/datarate_node/dst_car", dst_car);

    // setup message and service
    adhoc_customize::CarMsg testMsg;
    testMsg.dst_car = dst_car;
    car_communication::sendMessageService service;


    //some vars
    int i_count = 0;
    int failed = 0;
    double start;


    while(ros::ok()){

        if(i_count == max_i_count){
            printSummary(failed);
            return 0;
        }

        //start timer
        start = ros::Time::now().toSec();

        for(int i = 0; i<size; ++i){
            //call service
            service.request.msg = testMsg;

            client.call(service);

            if(!service.response.success){
                ++failed;
            }
            ros::spinOnce();
        }


        //stop timer and store time
        time_values.push_back(ros::Time::now().toSec() - start);

        ROS_INFO("finished loop %d ", i_count);
        ++i_count;

        ros::spinOnce();
    }


    return 0;

}

