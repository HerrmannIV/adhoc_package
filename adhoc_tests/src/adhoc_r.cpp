#include "ros/ros.h"
#include "adhoc_customize/include.h"
#include "std_msgs/String.h"
#include "adhoc_communication/RecvString.h"

void rectangleCallback(const adhoc_customize::Rectangle::ConstPtr& msg){
	ROS_INFO("I heard: [%d][%d]", msg->length, msg->width);
}

void StringCallback(const std_msgs::String::ConstPtr& msg){
	std::cout << "I heard string of legth: "<< msg->data.length() << " Bytes\n";
	//ROS_INFO("I heard string: [%s]", msg->data.c_str());
}

void stringCallback(const adhoc_communication::RecvString::ConstPtr& msg){
	std::cout << "I heard string of legth: "<< msg->data.length() << " Bytes\n";
	//ROS_INFO("I heard string: [%s]", msg->data.c_str());
}
int main(int argc, char **argv){
	ros::init(argc, argv, "adhoc_receiver_node");  
	ros::NodeHandle n;
	ros::Subscriber sub_r = n.subscribe("t_rectangle", 1000, rectangleCallback);  
	ros::Subscriber sub_s = n.subscribe("t_String", 1000, StringCallback);  
	ros::Subscriber sub_s2 = n.subscribe("t_string", 1000, stringCallback);  
	ros::spin();  
	return 0;
}