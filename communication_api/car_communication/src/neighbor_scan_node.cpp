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
#include <vector>


std::vector<std::string> neighbors;
std::vector<std::string> carsInSight;

/**
 * @brief vectorContains
 *      checks if v contains string s
 * @param v
 *      vector of strings
 * @param s
 *      string to search for
 * @return
 *      true if v contains s, else false
 */
bool vectorContains(std::vector<std::string> *v , std::string s){

    for(int i = 0; i < v->size(); ++i){
                if(v->at(i).compare(s) == 0)
                        return true;
        }
        return false;
}



/**
 * @brief tagCallback
 * Subscribtion to the ar_track package to get cars in sight
 * @param foundMarkers
 */
void tagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &foundMarkers){
    std::vector<std::string> result;
    std::vector<ar_track_alvar_msgs::AlvarMarker> received = foundMarkers->markers;

    int current_id;
    std::string current_name;

    for(int i =0 ; i<received.size(); ++i){
        current_id = received.at(i).id;
        if((current_id != 0) && (current_id <= 6)){
            result.push_back("pses-0" + std::to_string(current_id));
        }
    }
    carsInSight = result;
    return;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "neighbor_scan_node");
    ros::NodeHandle nh;

    //subscriber
    ros::Subscriber tags = nh.subscribe("/ar_pose_marker", 100, tagCallback);

    //publisher
    ros::Publisher carsInSightPub = nh.advertise<adhoc_customize::CarsInSight>("cars_in_sight",100);

    //note that this CarsInSight message is used because of implementing a string array message
    ros::Publisher neighborsPub = nh.advertise<adhoc_customize::CarsInSight>("neighbors",100);

    //client
    ros::ServiceClient client = nh.serviceClient<adhoc_communication::GetNeighbors>("/adhoc_communication/get_neighbors");
    adhoc_communication::GetNeighbors srv;

    adhoc_customize::CarsInSight cis;

    std::vector<std::string> neighbors;
    adhoc_customize::CarsInSight nb;

    ros::Rate loop_rate = 10;

    while(ros::ok()){

        if(client.call(srv)){
            neighbors = srv.response.neigbors;

                //publish neighbors
                nb.cars = neighbors;
                neighborsPub.publish(nb);

        }else{
            ROS_ERROR("get neighbors failed");
        }

        //publish cars in sight
        cis.cars = carsInSight;
        carsInSightPub.publish(cis);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
