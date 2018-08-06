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

std::vector<double> values;
std::vector<double> receivedID;


void msgCallback(const adhoc_customize::CarMsg::ConstPtr &receivedMsg){

    return;
}


void printSummary(int loops, int failed){

    double sum =0;
    double current;

    //output file
    std::ofstream stream;
    stream.open("/home/pses/Documents/test_results/bc_latency.csv", std::ofstream::out | std::ofstream::app);

    for (int i = 0; i<values.size();++i){
        current = values.at(i);
        stream << (std::to_string(current) + ";");
        sum += current;

    }
    stream.close();
    double average = sum/(loops-failed);

    ROS_INFO("failed: %d", failed);
    ROS_INFO("-----SUMMARY----- \n average latency: %f", average);



}



int main(int argc, char **argv){

    ros::init(argc, argv, "bc_latency");
    ros::NodeHandle nh;

    //subscribers
    ros::Subscriber msgSub = nh.subscribe("/recvMsg", 10, msgCallback);

    //service client
    ros::ServiceClient clientBC = nh.serviceClient<car_communication::broadcastMessageService>("broadcastMessageService");


    //params
  /*  int rate;
    nh.getParam("/bc_latency/rate", rate);
    ros::Rate loop_rate(rate);
*/
    int loops;
    nh.getParam("/bc_latency/loops", loops);


    // setup message and service
    adhoc_customize::CarMsg testMsg;
    testMsg.dst_car = "all";
    car_communication::broadcastMessageService service;


    int counter = 0;
    int failed = 0;

    double start;


    while(ros::ok()){

        if(counter == loops){
            break;
        }

        service.request.msg = testMsg;

        //set timer
        start = ros::Time::now().toSec();
        //call service
        clientBC.call(service);
        //store time value
        values.push_back(ros::Time::now().toSec() - start);


        if(!service.response.success){
            ++failed;
        }

        ++counter;
        ros::spinOnce();

    }


    printSummary(loops ,failed);

    return 0;

}

