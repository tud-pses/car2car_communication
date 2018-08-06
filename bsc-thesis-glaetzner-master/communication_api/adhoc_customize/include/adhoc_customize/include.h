
// DO NOT CHANGE
#define FRAME_DATA_TYPE_CUSTOM 0xf0

//Begin Custom with 0xF1, 0xF2...
#define FRAME_DATA_TYPE_CAR_MSG 0xf1
#define FRAME_DATA_TYPE_CARS_IN_SIGHT 0xf2
#include "adhoc_customize/CarMsg.h"
#include "adhoc_customize/CarsInSight.h"

#include "std_msgs/String.h"
#include "std_msgs/Time.h"

#ifdef COMM
void publishCustomMessage(std::string payload, std::string topic, uint8_t data_type, std::string src_host){

	if (data_type == FRAME_DATA_TYPE_CAR_MSG){
		ROS_INFO("FRAME_DATA_TYPE_CAR_MSG");
		adhoc_customize::CarMsg carMsg;
		desializeObject((unsigned char*) payload.data(), payload.length(), &carMsg);
		publishMessage(carMsg, topic);
	}else if (data_type == FRAME_DATA_TYPE_CARS_IN_SIGHT){
		ROS_INFO("FRAME_DATA_TYPE_CARS_IN_SIGHT");
		adhoc_customize::CarsInSight carsInSight;
		desializeObject((unsigned char*) payload.data(), payload.length(), &carsInSight);
		publishMessage(carsInSight, topic);
	}else
   
    ROS_ERROR("UNKNOWN FRAME_DATA_TYPE");
}
#endif

