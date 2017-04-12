#include "ros/ros.h"
#include "adhoc_customize/include.h"


int main (int argc, char **argv){
	
	ros::init(argc, argv, "dummy");
	ros::NodeHandle nh; 
	ros::Publisher confPub = nh.advertise<std_msgs::String>("t_filename", 1000, false);
	ros::spin();
}