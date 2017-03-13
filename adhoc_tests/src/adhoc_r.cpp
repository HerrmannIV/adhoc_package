#include "ros/ros.h"
#include "adhoc_customize/include.h"

void rectangleCallback(const adhoc_customize::Rectangle::ConstPtr& msg){
	ROS_INFO("I heard: [%d][%d]", msg->length, msg->width);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "adhoc_receiver_node");  
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("t_rectangle", 1000, rectangleCallback);  
	ros::spin();  
	return 0;
}