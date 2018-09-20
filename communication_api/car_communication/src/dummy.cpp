#include "ros/ros.h"

#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"

#include "car_communication/sendMessageService.h"
#include "car_communication/broadcastMessageService.h"

#include <list>
#include <stdlib.h>
#include <string>
#include <vector>

int count;

void msgCallback(const adhoc_customize::CarMsg::ConstPtr &receivedMsg){
    ROS_INFO("dummy message %f --- %d ", receivedMsg->msg_ID, count+1);
    count++;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "user_node");
    ros::NodeHandle nh;

    //subscribers
    ros::Subscriber msgSub = nh.subscribe("/msgs_to_user", 10, msgCallback);
    count = 0;
    ros::spin();
    return 0;

}

