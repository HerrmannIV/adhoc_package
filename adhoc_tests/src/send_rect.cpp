#include "ros/ros.h"
#include "ros/topic.h"
#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"
#include "adhoc_communication/SendString.h"
#include "adhoc_communication/RecvString.h"
#include <adhoc_tests/FilenameService.h>
#include <iostream> 
#include <fstream>
#include <string> 
#include <sys/time.h>


adhoc_customize::Rectangle rectangle;


int main (int argc, char **argv){
	
	ros::init(argc, argv, "adhoc_sender1");
	ros::NodeHandle nh; 

	// get Parameters and print INFO
	int loop;
	std::string dst_car, pos;
	nh.getParam("/rect/dst_car", dst_car);
	nh.getParam("/rect/loop", loop);
	ROS_INFO("loop [%d]; Dest: [%s]", loop, dst_car.c_str());
	
	ros::Rate loop_rate(1);


	int i = 0;

	while(ros::ok() && i<loop){
		i++;
		// send Rectangle
		rectangle.length = i;
		rectangle.width = i;
		adhoc_communication::sendMessage(rectangle, FRAME_DATA_TYPE_RECTANGLE, dst_car, "t_rectangle");
		loop_rate.sleep();
	}	
	return 1;
}