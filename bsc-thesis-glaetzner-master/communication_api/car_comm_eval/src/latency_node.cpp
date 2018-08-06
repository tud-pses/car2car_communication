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


bool isInVector(double value, std::vector<double> vector){
    for(int i = 0; i<vector.size(); ++i){
        if(vector.at(i) == value)
            return true;
    }
    return false;

}


void msgCallback(const adhoc_customize::CarMsg::ConstPtr &receivedMsg){

    if(isInVector(receivedMsg->msg_ID, receivedID)){
        return;
    }else{
        receivedID.push_back(receivedMsg->msg_ID);
    }

    //determine latency
    double now = ros::Time::now().toSec();
    if(receivedMsg->msg_type == "reply"){
        double latency = now - receivedMsg->msg_ID;
        //ROS_INFO("%f --------- %f", latency, receivedMsg->msg_ID);
        values.push_back(latency);
    }

    return;
}


void printSummary(int rate, int loops, int failed){

    double sum =0;
    double current;

    //output file
    std::ofstream stream;
    std::string filename = "/home/pses/Documents/test_results/latency_r" + std::to_string(rate) + "_nr.csv";
    stream.open(filename, std::ofstream::out | std::ofstream::app);

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

    ros::init(argc, argv, "latency_node");
    ros::NodeHandle nh;

    //subscribers
    ros::Subscriber msgSub = nh.subscribe("/recvMsg", 10, msgCallback);

    //service client
    ros::ServiceClient client = nh.serviceClient<car_communication::sendMessageService>("sendMessageService");

    //params
    int rate;
    nh.getParam("/latency_node/rate", rate);
    ros::Rate loop_rate(rate);

    int loops;
    nh.getParam("/latency_node/loops", loops);

    std::string dst_car;
    nh.getParam("/latency_node/dst_car", dst_car);

    // setup message and service
    adhoc_customize::CarMsg testMsg;
    testMsg.dst_car = dst_car;
    car_communication::sendMessageService service;


    int counter = 0;
    int failed = 0;

    ROS_INFO("starting process with rate %d and %d loops",rate, loops);

    while(ros::ok()){

        if(counter< loops){
            service.request.msg = testMsg;

            client.call(service);

            if(!service.response.success){
                ++failed;
            }

        }else{
            break;
        }

        ++counter;
        ros::spinOnce();
        loop_rate.sleep();
    }


    printSummary(rate, loops ,failed);

    return 0;

}

