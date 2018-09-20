#include "ros/ros.h"

#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"

#include "car_communication/sendMessageService.h"
#include "car_communication/broadcastMessageService.h"

#include "platooning/PlatoonMsg.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include <list>
#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>

int last_mode;
double last_zero;

int last_speed;
double last_speed_time;

std::vector<double> joins;
std::vector<double> leaves;

std::vector<double> join_req;
std::vector<double> leave_req;

std::vector<double> speed;
std::vector<double> speed_command;


void platoonInfoCallback(const platooning::PlatoonMsg::ConstPtr &msg){

    ROS_INFO("mode: %d",msg->mode);

    if(last_speed == msg->speed){
        last_speed_time = ros::Time::now().toSec();
    }else{
        ROS_INFO("speed %d -> %d" , last_speed, msg->speed);
        last_speed = msg->speed;
        last_speed_time = ros::Time::now().toSec();
        speed.push_back(last_speed_time);

    }


/*    double time = ros::Time::now().toSec();
    int current_mode = msg->mode;
    if(current_mode == 0){
        last_zero = ros::Time::now().toSec();
    }

    if(current_mode != last_mode){
            if(last_mode == 0){
                ROS_INFO("mode 0 -> 1 at: %f", time);
                joins.push_back(time - last_zero);
            }
            if(last_mode == 1){
                ROS_INFO("mode 1 -> 0 at: %f", time);
                leaves.push_back(time);
            }
    }

    last_mode = msg->mode;
*/
    return;

}


void printResults(){

    double current;

    //output file
    std::ofstream stream;
    stream.open("/home/pses/Documents/test_results/speed.csv", std::ofstream::out | std::ofstream::app);

    //print speed
    for (int i = 0; i<speed.size();++i){
        current = speed.at(i) - speed_command.at(i);
        stream << (std::to_string(current) + ";");
    }

    //print leaves
/*    stream << "leaves;";
    for (int i = 0; i<leaves.size();++i){
        current = leaves.at(i) - leave_req.at(i);
        stream << (std::to_string(current) + ";");
    }

    //print joins
    stream << "joins;" ;
    for (int i = 0; i<joins.size();++i){
        stream << (std::to_string(joins.at(i)) + ";");
    }
*/
    stream.close();

    return;

}


int main(int argc, char **argv){

    ros::init(argc, argv, "platoon_user_node");
    ros::NodeHandle nh;

    int rate = 10;
    nh.getParam("/platoon_user_node/rate", rate);


    //Publisher
    ros::Publisher currentSpeedPub =  nh.advertise<std_msgs::Int16>("currentSpeed",1);
    ros::Publisher enablePub = nh.advertise<std_msgs::Bool>("enablePlatooning",1);
    ros::Publisher leavePub =nh.advertise<std_msgs::Bool>("leavePlatoon",1);
    ros::Publisher setSpeedPub = nh.advertise<std_msgs::Int16>("setSpeed",1);
    ros::Publisher setDistancePub = nh.advertise<std_msgs::Float64>("setDistance",1);

    //Subscriber
    ros::Subscriber platoonPub = nh.subscribe("/platoon_info",1,platoonInfoCallback);

    double time = ros::Time::now().toSec();

    std_msgs::Bool leaveMsg;
    leaveMsg.data = true;

    std_msgs::Int16 speedMsg;
    speedMsg.data = 1;

    last_speed = 0;

    int counter = 0;
    last_mode = 0;

    while(ros::ok()){

        if(counter == 100){
            printResults();
            return 0;
        }

        if(ros::Time::now().toSec() - time > 2){
            time = ros::Time::now().toSec();

            setSpeedPub.publish(speedMsg);
            speed_command.push_back(time);

            counter++;
            speedMsg.data = counter +1;
            /*
            leavePub.publish(leaveMsg);
            leave_req.push_back(time);
            counter++;
            ROS_INFO("counter: %d", counter);
            */
        }



        ros::spinOnce();
    //  loop_rate.sleep();

    }
}
