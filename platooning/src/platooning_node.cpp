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

ros::ServiceClient *sendMsgClient_ptr;
ros::ServiceClient *bcMsgClient_ptr;


std::string my_name;
int my_id;

bool enablePlatooning;
bool platoon_mode;
bool platoon_master;

std::string master_name;
int master_id;
double last_master_contact;

std::string follow_name;
int follow_id;

std::string front_name;
int front_id;

int speed;
double distance;

std::vector<std::string> platoon;

bool processing;
std::vector<std::string> visibleCars;

/**
 * @brief resetPlatoonData
 *      resets the platoon data to its default data
 */
void resetPlatoonData(){
    platoon_mode = false;
    platoon_master = false;

    master_name.erase();
    master_id = 0;

    follow_name.erase();
    follow_id = 0;

    front_name.erase();
    front_id = 0;

    speed = 0;
    distance = 0;

    platoon.clear();
    return;
}

/**
 * @brief sendMsg
 *      sends the given message by using the car_communication::sendMessageService, retrying it five times
 * @param msg
 *      message to send
 * @return success
 *      by sending the message, true if successful else false
 */
bool sendMsg(adhoc_customize::CarMsg msg){
	
	//init service
	car_communication::sendMessageService service;
	service.request.msg = msg;
	
        if(msg.command.compare("ping") != 0){
            ROS_INFO("message: -- %s -- send to -- %s -- ", msg.command.c_str(), msg.dst_car.c_str());
        }

	//send message
	int count = 0;
        while(count < 5 ){
                sendMsgClient_ptr->call(service);
		if(service.response.success){
			return true;
		}
		++count;
		
	}
	
	return false;
}


/**
 * @brief giveReply
 *      sends a positive or negativ reply for a platoon request
 * @param dst
 *      destination car
 * @param pos_neg
 *      true if reply ist positive, false if joining platoon is denied
 */
void giveReply(const std::string dst, bool pos_neg){
	
	//init reply message
	adhoc_customize::CarMsg reply;
	reply.dst_car = dst;
	if(pos_neg)
		reply.command = "platoon_rep_pos";
	else
		reply.command = "platoon_rep_neg";
	
	sendMsg(reply);
	return;
				
}


/**
 * @brief giveSpeed
 *      send speed to a car which should be in the platoon
 * @param dst
 *      destination
 * @param s
 *      speed
 */
void giveSpeed(const std::string dst, int s){
	
	adhoc_customize::CarMsg msg;
	msg.dst_car = dst;
	msg.command = "platoon_speed";
	msg.data = s;
	
	sendMsg(msg);
	return;
}


/**
 * @brief giveDistance
 *      send distance to a car which should be held between the cars in the platoon
 * @param dst
 *      destination car
 * @param d
 *      distance
 */
void giveDistance(const std::string dst, double d){
		
	adhoc_customize::CarMsg msg;
	msg.dst_car = dst;
	msg.command = "platoon_distance";
	msg.data = d;
	
	sendMsg(msg);
	return;
}


/**
 * @brief giveFront
 *      sends the information about the front car to an other car, the data field contains the ID of the front car
 * @param dst
 *      destination car
 * @param front
 *      car name of the following car
 */
void giveFront(const std::string dst, const std::string front){
	
	adhoc_customize::CarMsg msg;
	msg.dst_car = dst;
	msg.command = "platoon_front";
        msg.data = atoi(&front.back());
	
	sendMsg(msg);
	return;
}


/**
 * @brief giveFollow
 *      sends the information about the following car to an other car, the data field contains the ID of the follow car
 * @param dst
 *      destination car
 * @param follow
 *      car name of the following car
 */
void giveFollow(const std::string dst, const std::string follow){
	
	adhoc_customize::CarMsg msg;
	msg.dst_car = dst;
	msg.command = "platoon_follow";
        msg.data = atoi(&follow.back());
	
	sendMsg(msg);
	return;
}


/**
 * @brief giveSightError
 *      sends a sight error message to the platoon master
 *      the message data field contains the car ID that is not visible
 */
void giveSightError(){
	adhoc_customize::CarMsg msg;
	msg.command = "platoon_sight_error";
	msg.dst_car = master_name;
	msg.data = front_id;
	
	sendMsg(msg);
	return;
}


/**
 * @brief givePing
 *      sends a ping message to a car
 * @param dst
 *      destination car of the message
 * @return
 *      true if contact was successful else false
 */
bool givePing(const std::string dst){
    adhoc_customize::CarMsg msg;
    msg.dst_car = dst;
    msg.command = "ping";
    return sendMsg(msg);

}


/**
 * @brief giveExit
 *      sends an exit command to a car
 * @param dst
 *      destination car of the message
 */
void giveExit(const std::string dst){
    adhoc_customize::CarMsg msg;
    msg.dst_car = dst;
    msg.command = "platoon_exit";

    sendMsg(msg);
    return;
}


/**
 * @brief giveMember
 *      sends the id of a platoon member to an other car
 * @param dst
 *      destination car
 * @param id
 *      id of the member
 */
void giveMember(const std::string dst, int id){
    adhoc_customize::CarMsg msg;
    msg.dst_car = dst;
    msg.data = id;
    msg.command = "platoon_member";

    sendMsg(msg);
    return;
}


/**
 * @brief giveMaster
 *      sends new master command to a car
 * @param dst
 *      destination car
 * @param id
 *      new master's id
 */
void giveMaster(const std::string dst, int id){
    adhoc_customize::CarMsg msg;
    msg.dst_car = dst;
    msg.data = id;
    msg.command = "platoon_new_master";

    sendMsg(msg);
    return;
}


/**
 * @brief giveLeaving
 *      sends a leaving message to the platoon master
 */
void giveLeaving(){
    adhoc_customize::CarMsg msg;
    msg.dst_car = master_name;
    msg.command = "platoon_leaving";

    sendMsg(msg);
    return;
}


/**
 * @brief giveMasterLeave
 *      handles the exit of the platoon master
 */
void giveMasterLeave(){

    //give platoon member to new master
    for(int i = 1; i<platoon.size(); ++i){
        giveMember(follow_name, atoi(&platoon.at(i).back()));
    }
    //tell any member new master
    for(int i = 0; i<platoon.size(); ++i){
        giveMaster(platoon.at(i), follow_id);
    }
    return;
}


/**
 * @brief vectorPos
 *      determines the position of a string in a vector
 * @param v
 *      vector
 * @param s
 *      string
 * @return
 *      position of the s in v
 */
int vectorPos(std::vector<std::string> *v, std::string s){

    for(int i = 0; i < v->size(); ++i){
        if(v->at(i).compare(s) == 0){
            return i;
        }else{
            continue;
        }
    }
    return 0;

}


/**
 * @brief vectorRemove
 *      removes a string in a vector
 * @param v
 *      vector
 * @param s
 *      string to be removed
 * @return
 *      vector without the removed element
 */
std::vector<std::string> vectorRemove(std::vector<std::string> *v, std::string s){

    std::vector<std::string> new_platoon;
    for(int i = 0; i < v->size(); ++i){
        if(v->at(i).compare(s) == 0){
            continue;
        }else{
            new_platoon.push_back(v->at(i));
        }
    }
    return new_platoon;

}


/**
 * @brief vectorContains
 *      determines if a vector contains string
 * @param v
 *      vector
 * @param s
 *      string to be searched for
 * @return
 *      true if v contains s otherwise return false
 */
bool vectorContains(std::vector<std::string> *v , std::string s){

    for(int i = 0; i < v->size(); ++i){
                if(v->at(i).compare(s) == 0)
                        return true;
        }
        return false;
}


/**
 * @brief removeCarFromPlatoon
 *      removes a car from the platoon, more detail in the code
 * @param remove_car
 *      car name which shall be removed
 */
void removeCarFromPlatoon(std::string remove_car){

    if(!vectorContains(&platoon,remove_car)){
        return;
    }

    //determine position
    int remove_pos = vectorPos(&platoon,remove_car);
    int platoon_size= platoon.size();

    //tell removed car that it will be removed
    giveExit(remove_car);

    /*
     *  if remove_car is at position 0 (behind the master) -> remove it
     *   giveFront "master" to the new 0 if there is one
     *   and set own follow to this new follower
     */
    if(remove_pos == 0){
            platoon = vectorRemove(&platoon,remove_car);
            if(platoon.size() !=0){
                giveFront(platoon.front(), master_name);
                follow_name = platoon.front();
                follow_id = atoi(&follow_name.back());
            }else{
                follow_name.erase();
                follow_id =0;
            }

            return;

    /*
     *  if remove car is last in platoon -> remove it
     *   giveFollow 0 to new last
     */
    }else if(remove_pos == platoon_size -1){
        platoon.pop_back();
        if(platoon.size() != 0){
            giveFollow(platoon.back(), "0");
        }
        return;



    /*
     *  if remove_car is anywhere in the platoon -> determine front and follow, remove it,
     *  giveFront to follower and giveFollow to front
     */
    }else{
        std::string front = platoon.at(remove_pos -1);
        std::string follow =platoon.at(remove_pos +1);
        platoon = vectorRemove(&platoon,remove_car);
        giveFollow(front, follow);
        giveFront(follow, front);
    }


}


/**
 * @brief destroyPlatoon
 *      sends a exit message to any platoon member and resets its platoon data
 */
void destroyPlatoon(){
    for(int i = 0; i < platoon.size(); ++i){
        giveExit(platoon.at(i));
    }
    resetPlatoonData();
    return;
}


/**
 * @brief askMaster
 *      forwards a message to the platoon master
 * @param msg
 *      message to be forwarded
 */
void askMaster(adhoc_customize::CarMsg msg){
	
	msg.dst_car = master_name;
	
	sendMsg(msg);
	return;
	
}


/**
 * @brief requestPlatooning
 *      sends a platooning request to a visible car
 */
void requestPlatooning(){
	adhoc_customize::CarMsg msg;
	msg.dst_car = visibleCars.front();
	msg.command = "platoon_req";
        msg.data = my_id;   //when request to join add own id to the data field (forwarding to master)

	if(sendMsg(msg))
                processing =true;
	return;
}


/**
 * @brief checkSightFront
 *      checks if the car which should be in front is visible
 * @param res
 *      used as second return result
 * @return
 *      Returns true if the car in front is in sight and stores the the time of this sight contact in the param *res.
 *      Returns false if there is no sight contact.
 */
bool checkSightFront(double *res){
        if(vectorContains(&visibleCars, front_name)){
		*res = ros::Time::now().toSec();	
                return true;
	}else{
                return false;
	}	
}


/**
 * @brief leavePlatoon
 *      sends a message to the master to leave the platoon and resets the platoon data
 */
void leavePlatoon(){
    if(!platoon_master){
        giveLeaving();
        resetPlatoonData();
    }else{
        giveMasterLeave();
        resetPlatoonData();
    }
    return;
}


/**
 * @brief printStatus
 *      prints the status of the car in its platoon
 */
void printStatus(){
    ROS_INFO("mode: %d --- master: %d --- front_id: %d --- follow_id: %d", platoon_mode, platoon_master, front_id, follow_id);
    if(platoon_master){
        std::string members;
        for(int i = 0; i<platoon.size(); ++i){
            members = members + platoon.at(i) + " ; ";
        }
        ROS_INFO("platoon: %s", members.c_str());
    }
    return;
}


/**
 * @brief msgCallback
 *      Callback function for the incoming messages from other cars which are sent by using the car_communication package.
 *      It decides what to do depending on the content of the command of the incomming message. More details in the code itself.
 * @param receivedMsg
 *      incomming message on this topic
 */
void msgCallback(const adhoc_customize::CarMsg::ConstPtr &receivedMsg){
	std::string command = receivedMsg->command;
	adhoc_customize::CarMsg msg = *receivedMsg;

        if(command.compare("platoon_req") == 0){
            if(platoon_mode){
                if(platoon_master){
                    std::string dst = "pses-0" + std::to_string((int)msg.data);
                    if(vectorContains(&platoon, dst )){
                        ROS_INFO("ignored request");
                        return;
                    }
                    //give platoon information to the new member
                    giveReply(dst, true);
                    giveSpeed(dst, speed);
                    giveDistance(dst, distance);
                    giveFront(dst, platoon.back());
                    giveFollow(platoon.back(), dst);
                    platoon.push_back(dst);
                }else{
                    //forwarding message to the platoon_master
                    ROS_INFO("ask master");
                    askMaster(msg);
                }
            }else{
                std::string dst = "pses-0" + std::to_string((int)msg.data);
                //reject request if platooning is disabled
                if(!enablePlatooning){
                    giveReply(dst, false);
                    return;
                }
                //create a new platoon as master
                ROS_INFO("create platoon");
                platoon_mode =true;
                platoon_master = true;
                master_id = my_id;
                master_name = my_name;
                giveReply(dst, true);
                giveSpeed(dst, speed);
                giveDistance(dst, distance);
                giveFront(dst, my_name);
                follow_name = dst;
                follow_id = (int)msg.data;
                platoon.push_back(dst);
            }
            printStatus();
            return;


        }else if(command.compare("platoon_rep_pos")  == 0){
            //positive reply on a platooning request
            ROS_INFO("got positive reply");
            platoon_mode = true;
            platoon_master = false;
            master_name = msg.src_car;
            master_id = atoi(&master_name.back());
            processing = false;
            printStatus();
            return;


        }else if(command.compare("platoon_rep_neg") == 0){
            //negative reply on a platooning request
            processing = false;
            return;


        }else if(command.compare("platoon_speed") == 0){
            ROS_INFO("got speed");
            if(msg.src_car == master_name)
                    speed = msg.data;
            return;


        }else if(command.compare("platoon_distance") == 0){

            ROS_INFO("got distance");
            if(msg.src_car == master_name)
                    distance = msg.data;
            return;


        }else if(command.compare("platoon_front") == 0){
            //master tells which car is the one in front
            if(msg.src_car == master_name){
                    front_name = "pses-0" + std::to_string((int)msg.data);
                    front_id = (int)msg.data;
                    ROS_INFO("got new front: %s", front_name.c_str());
            }
            printStatus();
            return;


        }else if(command.compare("platoon_follow") == 0){
            //master tells which car is the one behind
            ROS_INFO("got follow");
            if(msg.src_car == master_name){

                if(msg.data == 0){
                    //delete follower
                    follow_id = 0;
                    follow_name.erase();
                }else{
                    //set new follower
                    follow_name = "pses-0" + std::to_string((int)msg.data);
                    follow_id = (int)msg.data;
                }
            }
            printStatus();
            return;


        }else if(command.compare("platoon_sight_error") == 0){
            //information for master that there is a member out of sight
            ROS_INFO("got sight error");
            if(msg.data == master_id){
                destroyPlatoon();
                return;
            }
            removeCarFromPlatoon("pses-0"+std::to_string((int)msg.data));
            printStatus();
            return;



        }else if(command.compare("platoon_exit") == 0){
            //command given by the master to exit the platoon
            ROS_INFO("got exit");
            resetPlatoonData();
            printStatus();
            return;

        }else if(command.compare("platoon_leaving") == 0){
            //information for the master if a car wants to leave the platoon by itself
            ROS_INFO("got leaving");
            removeCarFromPlatoon(msg.src_car);
            if(platoon.size() == 0){
                resetPlatoonData();
            }
            printStatus();
            return;

        }else if(command.compare("ping") == 0){
            //checking network connectivity
            last_master_contact = ros::Time::now().toSec();
            return;
        }else if(command.compare("platoon_member") == 0){
            ROS_INFO("got platoon member");
            if(msg.src_car == master_name){
                platoon.push_back("pses-0" + std::to_string((int)msg.data));
            }
            printStatus();
            return;

        }else if(command.compare("platoon_new_master") == 0){
            ROS_INFO("got new master");
            if(msg.src_car == master_name){
                if(msg.data == my_id){
                    platoon_master = true;
                    front_id = 0;
                    front_name.erase();
                }
                master_name = "pses-0" + std::to_string((int)msg.data);
                master_id = (int)msg.data;
            }
            printStatus();
            return;

        }else{
            return;
        }

}


/**
 * @brief carsCallback
 *      callback function for the CarsInSight message which are provided by the car_communication package
 * @param msg
 *      incomming message on this topic
 */
void carsCallback(const adhoc_customize::CarsInSight::ConstPtr &msg){
        visibleCars = msg->cars;
        return;
}


/**
 * @brief speedCallback
 *      sets the platooning speed to current speed provided by the user, so that it can be used
 *      if this car gets master role in a platoon
 * @param msg
 *      incoming message with speed in its data field
 */
void speedCallback(const std_msgs::Int16::ConstPtr &msg){

    if(platoon_mode){
        return;
    }else{
        if(msg->data >= 0 && msg->data <= 1000){
            speed = msg->data;
        }
    }
}


/**
 * @brief enableCallback
 *      enables platooning
 * @param msg
 *      true or false in the data field for enable or disable
 */
void enableCallback(const std_msgs::Bool::ConstPtr &msg){
    enablePlatooning = msg->data;
    return;
}


/**
 * @brief leaveCallback
 *      user can give the leave command
 * @param msg
 *      true in data field if platoon should be leaved
 */
void leaveCallback(const std_msgs::Bool::ConstPtr &msg){
    if(msg->data){
        leavePlatoon();
    }
    return;
}

/**
 * @brief setSpeedCallback
 *      sets the platoon's speed to the desired value, if this is the master
 * @param msg
 *      new speed
 */
void setSpeedCallback(const std_msgs::Int16::ConstPtr &msg){

    if(platoon_mode && platoon_master){

        int newSpeed = msg->data;
        if(newSpeed >= 0 && newSpeed <= 1000){
            speed = newSpeed;
            //communicate to all
            for(int i = 0; i<platoon.size(); ++i){
                giveSpeed(platoon.at(i),newSpeed);
            }
        }
    }
    return;
}

/**
 * @brief setDistanceCallback
 *      sets the platoon's distance to the desired value, if this is the master
 * @param msg
 *      new distance
 */
void setDistanceCallback(const std_msgs::Float64::ConstPtr &msg){

    if(platoon_mode && platoon_master){
        double newDistance = msg->data;
        distance = newDistance;
        //communicate to all
        for(int i = 0; i<platoon.size(); ++i){
            giveDistance(platoon.at(i),newDistance);
        }
    }
    return;
}






/**
 * @brief setPlatoonInfo
 *      updates the platoon information for the outgoing topic
 * @param info
 *      pointer to a PlatoonMsg that will be publish for user
 */
void setPlatoonInfo(platooning::PlatoonMsg *info){
    info->mode = platoon_mode;
    info->master = master_name;
    info->follow = follow_name;
    info->front = front_name;
    info->speed = speed;
    info->distance = distance;
    info->platoon = platoon;
    return;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "platooning_node");
    ros::NodeHandle nh;
	

    //set Params
    nh.getParam("/car_communication_node/car_name",my_name);
    my_id = atoi(&my_name.back());

    double sight_timeout = 4;
    nh.getParam("/platooning_node/sight_timeout", sight_timeout);

    int rate = 5;
    nh.getParam("/platooning_node/rate", rate);

    double network_timeout = 4;
    nh.getParam("/platooning_node/network_timeout", network_timeout);


    //publisher
    ros::Publisher platoonPub = nh.advertise<platooning::PlatoonMsg>("platoon_info",1);
    ros::Publisher currentSpeedPub =  nh.advertise<std_msgs::Int16>("currentSpeed",1);
    ros::Publisher enablePub = nh.advertise<std_msgs::Bool>("enablePlatooning",1);
    ros::Publisher leavePub =nh.advertise<std_msgs::Bool>("leavePlatoon",1);
    ros::Publisher setSpeedPub = nh.advertise<std_msgs::Int16>("setSpeed",1);
    ros::Publisher setDistancePub = nh.advertise<std_msgs::Float64>("setDistance",1);


    //subscribers
    ros::Subscriber msgSub = nh.subscribe("/msgs_to_user", 10, msgCallback);
    ros::Subscriber carsSub = nh.subscribe("/cars_in_sight", 10, carsCallback);
    ros::Subscriber currentSpeedSub = nh.subscribe("/currentSpeed", 10, speedCallback);
    ros::Subscriber enableSub = nh.subscribe("/enablePlatooning", 10, enableCallback);
    ros::Subscriber leaveSub = nh.subscribe("/leavePlatoon", 10, leaveCallback);
    ros::Subscriber setSpeedSub = nh.subscribe("/setSpeed", 10, setSpeedCallback);
    ros::Subscriber setDistanceSub = nh.subscribe("/setDistance", 10, setDistanceCallback);


    //service clients
    ros::ServiceClient sendMsgClient = nh.serviceClient<car_communication::sendMessageService>("sendMessageService");
    ros::ServiceClient bcMsgClient = nh.serviceClient<car_communication::broadcastMessageService>("broadcastMessageService");

    //set client pointers
    sendMsgClient_ptr = &sendMsgClient;
    bcMsgClient_ptr = &bcMsgClient;

    //set default values for platooning data
    enablePlatooning = true;
    platoon_mode = false;
    platoon_master = false;
    processing = false;

    double last_sight;

    platooning::PlatoonMsg platoon_info;

    ros::Rate loop_rate(rate);
    while(ros::ok()){

        if(platoon_mode){

            if(platoon_master){

                //ping all platoon members
                for(int i = 0; i<platoon.size(); ++i){

                    //if failure again remove from platoon
                    if(givePing(platoon.at(i))){
                        continue;
                    }else{
                        removeCarFromPlatoon(platoon.at(i));
                    }

                }
                if(platoon.size() == 0){
                    resetPlatoonData();
                }

            }else{
                //check if front is in sight, store last sight contact
                if(!checkSightFront(&last_sight)){
                    if((ros::Time::now().toSec() - last_sight) > sight_timeout){ //check elapsed time
                        //inform master
                        ROS_ERROR("%f",ros::Time::now().toSec() - last_sight);
                        giveSightError();
                        last_sight = ros::Time::now().toSec();
                    }

                }
                //if there is no network connectivity to the master -> "leave" the platoon
                if(ros::Time::now().toSec() - last_master_contact > network_timeout){
                    resetPlatoonData();
                }

            }
        }else{
            if(!visibleCars.empty() && !processing && enablePlatooning){
                requestPlatooning();
                last_sight = ros::Time::now().toSec();
                last_master_contact = ros::Time::now().toSec();
            }
        }

        setPlatoonInfo(&platoon_info);
        platoonPub.publish(platoon_info);

        loop_rate.sleep();
        ros::spinOnce();
    }

return 0;

}

