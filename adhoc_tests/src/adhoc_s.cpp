#include "ros/ros.h"
#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"

int main (int argc, char **argv){
	
	ros::init(argc, argv, "adhoc_sender1");
	ros::NodeHandle nh;
	int rate;

	nh.getParam("/rate", rate);
	ROS_INFO("sending at rate %d Hz", rate);
	ros::Rate loop_rate(rate);
	int count = 0;

	adhoc_customize::Rectangle rectangle;
	int i = 0;
	rectangle.length = i;
	rectangle.width = i;

	ros::Time begin = ros::Time::now();
	std_msgs::String str; 
	str.data = "0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789"; //100Bytes

	while(ros::ok()){
		std::string dst_robot;
		nh.getParam("/dest_robot", dst_robot);
		//adhoc_communication::sendMessage(rectangle, FRAME_DATA_TYPE_RECTANGLE, dst_robot, "t_rectangle");
		adhoc_communication::sendMessage(str, FRAME_DATA_TYPE_STRING, dst_robot, "t_string");
		loop_rate.sleep();

		if (i<=100) i++;
		else {
			ros::Time end = ros::Time::now();
			ros::Duration dur = end-begin;
			ROS_INFO("dur: %f",dur.toSec());		
			return 1; //i = 65;
		}
		rectangle.length = i;
		rectangle.width = i;

	}
	return 1;
}

