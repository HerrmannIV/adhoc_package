#include "ros/ros.h"
#include "adhoc_customize/include.h"
#include "std_msgs/String.h"
#include "adhoc_communication/RecvString.h"
#include "adhoc_communication/SendString.h"
#include "adhoc_communication/functions.h"
#include "adhoc_communication/ChangeMCMembership.h"

#define JOIN 1
#define LEAVE 0


std::string command = "audacious -pq ~/Music/Beep-sound.ogx";


void rectangleCallback(const adhoc_customize::Rectangle::ConstPtr& msg){
	ROS_INFO("I heard: [%d][%d]", msg->length, msg->width);
}



void stringSerializedCallback(const adhoc_customize::StringWTime::ConstPtr& msg){
	std::cout << "I heard string of length: "<< msg->data.length() << " Bytes\n";
	std_msgs::Time time; 
	time.data = msg->time;	  
	adhoc_communication::sendMessage(time, FRAME_DATA_TYPE_TIME, msg->src_car, "t_answer");
}


void pingCallback(const adhoc_customize::RecvTime::ConstPtr& msg){
	ROS_INFO("I heard a Ping");	
	// Beep 
	//system(command.data());

	std_msgs::Time time; 
	time.data = msg->time;	  
	adhoc_communication::sendMessage(time, FRAME_DATA_TYPE_TIME, msg->src_car, "t_answer");
}





void stringServiceCallback(const adhoc_communication::RecvString::ConstPtr& msg){
	std::cout << "I heard string of length: "<< msg->data.length() << " Bytes\n";
	//ROS_INFO("I heard string: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
	ros::init(argc, argv, "adhoc_receiver_node");  
	ros::NodeHandle nh;
	ros::Subscriber sub_rect = nh.subscribe("t_rectangle", 1000, rectangleCallback);  
	ros::Subscriber sub_stringSerialized = nh.subscribe("t_sswt", 1000, stringSerializedCallback);  
	ros::Subscriber sub_stringService = nh.subscribe("t_stringServiceWTime", 1000, stringServiceCallback);  
	ros::Subscriber sub_ping = nh.subscribe("t_ping", 1000, pingCallback);  
	ros::ServiceClient client = nh.serviceClient<adhoc_communication::ChangeMCMembership>("ChangeMCMembership");
	adhoc_communication::ChangeMCMembership changeMC;
	changeMC.request.action = JOIN;
	changeMC.request.group_name = "mc_pses-car3";
	if (client.call(changeMC)){
    	ROS_INFO("Status: %d", changeMC.response.status);
  	}else{
    	ROS_ERROR("Failed to call service ChangeMCMembership");
  	}

	ros::spin();  
	return 0;
}


