#include "ros/ros.h"

#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"
#include "adhoc_communication/GetNeighbors.h"

#include "car_communication/sendMessageService.h"
#include "car_communication/broadcastMessageService.h"

#include "ar_track_alvar_msgs/AlvarMarker.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include <list>
#include <stdlib.h>
#include <string>
#include <algorithm>
#include <vector>

std::string car_name;
double retry_timeout;

std::list<double> activeRequest;
std::list<double> waitForReply;
std::list<double> waitForAck;

ros::Publisher outgoingPub;

std::vector<std::string> neighbors;
std::vector<std::string> carsInSight;

int counter;

/**
 * @brief isInList
 * checks if the list contains an element that equals value
 * @param list
 * @param value
 * @return
 * true if the object can be found in the lsit
 * false if not
 */
bool isInList(std::list<double> list, double value){

    if (std::find(list.begin(), list.end(), value) != list.end())
        return true;
    else
        return false;

}

/**
 * @brief isActiveRequest
 * checks if a request with id is already executing
 * @param id
 * @return
 */
bool isActiveRequest(double id){
    if(isInList(activeRequest, id))
        return true;
    else
        return false;
}

bool vectorContains(std::vector<std::string> *v , std::string s){

    for(int i = 0; i < v->size(); ++i){
        if(v->at(i).compare(s) == 0)
            return true;
    }
    return false;
}

/**
 * @brief recvMsgCallback
 * This function processes on the received message depending on its type. If it is a request
 * the function tries to send a reply periodically until timeout happens or the acknowledge message arrives.
 * While waiting for the reply, the ID will rest in the waitingForReply list.
 * If the received message is an reply, this function adds the ID of the reply message to the waitingForAck list
 * and removes the ID out of the waitForReply list.
 * If an acknowledge arrives it simply removes the ID out of the waitForAck list
 *
 * @param receivedMsg
 */
void recvMsgCallback(const adhoc_customize::CarMsg::ConstPtr &receivedMsg){

    ++counter;

    adhoc_customize::CarMsg msg = *receivedMsg;

    adhoc_customize::CarMsg msgToSend = msg;



     if(msg.msg_type.compare("request") == 0){

        //prevent multiple execution
        if(isActiveRequest(msgToSend.msg_ID)){
            return;
        }else{
            activeRequest.push_back(msgToSend.msg_ID);
        }

        outgoingPub.publish(msg);

        msgToSend.msg_type = std::string("reply");
        adhoc_communication::sendMessage(msgToSend, FRAME_DATA_TYPE_CAR_MSG, msg.src_car, "recvMsg");

        ++counter;

        //put ID in the waiting list
        waitForAck.push_back(msg.msg_ID);

        //init timer
        double elapsed_time;
        int max_retry = 10;
        int i = 0;
        double timer_start = ros::Time::now().toSec();

        //as long as the ID is in the waiting queue resend the reply
        while(isInList(waitForAck, msg.msg_ID)){

            elapsed_time = ros::Time::now().toSec() - timer_start;
            if(elapsed_time > retry_timeout){
                adhoc_communication::sendMessage(msgToSend, FRAME_DATA_TYPE_CAR_MSG, msg.src_car, "recvMsg");
                ++counter;
                ++i;
            }
            if(i > max_retry){
                waitForAck.remove(msg.msg_ID);
                activeRequest.remove(msg.msg_ID);
                break;
            }
            ros::spinOnce();
        }

        return;


    }else if(msg.msg_type.compare("reply") == 0){
        waitForReply.remove(msg.msg_ID);
        return;


    }else if(msg.msg_type.compare("ack") == 0){
        waitForAck.remove(msg.msg_ID);
        activeRequest.remove(msg.msg_ID);
        return;

    }


    return;

}



/**
 * @brief sendMessageService
 * This is the callback function for the sendMessageService.
 * It sends the message to the destinated car periodically until the recvMsgCallback() signals
 * that a reply to this request has arrived.
 * @param req
 * contains the message to be sent and its destination
 * @param res
 * contains a boolean which gives the information about the success for sending the message
 * @return
 */
bool sendMessageService(car_communication::sendMessageService::Request &req, car_communication::sendMessageService::Response &res){

    adhoc_customize::CarMsg msgToSend = req.msg;
    msgToSend.msg_type = "request";
    msgToSend.src_car = car_name;

    //check if destination is reachable
    if(!vectorContains(&neighbors, msgToSend.dst_car)){
        res.success = true;
        return true;
    }

    double id = ros::Time::now().toSec();
    msgToSend.msg_ID = id;

    //add request id to the waiting queue
    waitForReply.push_back(id);

    //send msg to dst_car
    adhoc_communication::sendMessage(msgToSend, FRAME_DATA_TYPE_CAR_MSG, msgToSend.dst_car, "recvMsg");

    ++counter;

    double timer_start = ros::Time::now().toSec();
    double elapsed_time;
    int max_retry = 10;
    int i = 0;

    while(isInList(waitForReply,msgToSend.msg_ID)){

        elapsed_time = ros::Time::now().toSec() - timer_start;

        if(elapsed_time > retry_timeout){
            adhoc_communication::sendMessage(msgToSend, FRAME_DATA_TYPE_CAR_MSG, msgToSend.dst_car, "recvMsg");

            ++counter;

            timer_start = ros::Time::now().toSec();
            ++i;
        }
        if(i>max_retry){
            res.success =false;
            waitForReply.remove(id);
            return true;
         }
        ros::spinOnce();
    }

    waitForReply.remove(id);

    msgToSend.msg_type = "ack";
    adhoc_communication::sendMessage(msgToSend, FRAME_DATA_TYPE_CAR_MSG, msgToSend.dst_car, "recvMsg");

    ++counter;

    res.success = true;
    return true;
}



/**
 * @brief broadcastMessageService
 * This is the callback for the broadcastMessageService. It sends the message given in req to all the car's neighbors
 * which are connected directly
 * @param req
 * @param res
 * @return
 */
bool broadcastMessageService(car_communication::broadcastMessageService::Request &req, car_communication::broadcastMessageService::Response &res){
    car_communication::sendMessageService service;
    service.request.msg = req.msg;
    service.request.msg.src_car = car_name;

    for(int i = 0; i<neighbors.size(); ++i){
        std::string current = neighbors.at(i);
        service.request.msg.dst_car = current;

        if(sendMessageService(service.request, service.response)){
            if(service.response.success){
                res.success = true;
            }
        }else{
            res.success = false;
            res.failed_dst.push_back(current);
        }
    }

    return true;
}


void neighborCallback(const adhoc_customize::CarsInSight::ConstPtr &msg){

    neighbors = msg->cars;

}

void carsInSightCallback(const adhoc_customize::CarsInSight::ConstPtr &msg){

    carsInSight = msg->cars;

}


int main(int argc, char **argv){

    ros::init(argc, argv, "car_communication_node");
    ros::NodeHandle nh;

    nh.getParam("/car_communication_node/car_name",car_name);
    nh.getParam("/car_communication_node/retry_timeout",retry_timeout);

    if(car_name.find("pses-0") == std::string::npos ){
        ROS_ERROR("WRONG CAR NAME FORMAT. MUST BE pses-0x. ALSO MAKE SURE THAT HOSTNAME LOOKS THE SAME");
        return 0;
    }

    //publisher
    outgoingPub = nh.advertise<adhoc_customize::CarMsg>("msgs_to_user", 100);
    ros::Publisher recvMsgPub = nh.advertise<adhoc_customize::CarMsg>("recvMsg", 100);

    //subscriber
    ros::Subscriber recvMsgSub = nh.subscribe("/recvMsg", 100, recvMsgCallback);
    ros::Subscriber neighborSub = nh.subscribe("/neighbors",100,neighborCallback);
    ros::Subscriber carsInSightSub = nh.subscribe("/cars_in_sight",100,carsInSightCallback);

    //services
    ros::ServiceServer sendMessageSrv = nh.advertiseService("sendMessageService", sendMessageService);
    ros::ServiceServer broadcastMessageSrv = nh.advertiseService("broadcastMessageService", broadcastMessageService);


    counter = 0;

     while(ros::ok()){

         ros::spinOnce();

     }

    return 0;
}

