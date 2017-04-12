#include "ros/ros.h"
#include "ros/topic.h"
#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"
#include "adhoc_communication/SendString.h"
#include "adhoc_communication/RecvString.h"
#include <adhoc_tests/FilenameService.h>
#include <adhoc_tests/sendPing.h>
#include <iostream> 
#include <fstream>
#include <string> 
#include <sys/time.h>


enum Mode{
	PING,
	STRING_SERIALIZE,
	STRING_SERVICE,
	RECT,
	STUD,
	PING_ALT
};

ros::Time beforePing;
bool ping;
std::ofstream outFile;
adhoc_customize::Rectangle rectangle;
std::string longstring;
int recvCounter = 0, linecounter = 0, i = 0;
int loop, mode_i, strLen;
std::string dst_car, pos;



bool sendPing(adhoc_tests::sendPing::Request &req, adhoc_tests::sendPing::Response &res){
	if(i <= loop){
		std_msgs::Time timeMsg;
		timeMsg.data = ros::Time::now();
		adhoc_communication::sendMessage(timeMsg, FRAME_DATA_TYPE_TIME, dst_car, "t_ping");
		return true;
	}
	return false;
}


void convertToPrefixString(int input, std::string &output){
	// convert int input to string
	std::ostringstream Stream;
    Stream << input;
	std::string len_s = Stream.str();
	
	// set prefix and crop amount
	std::string prefix = ""; int crop=0;
	if (input >=1000) { prefix = "k"; crop=3;}
	if (input >=1000000) { prefix = "M"; crop=6;}

	// crop and add prefix
	len_s.erase(len_s.end()-crop, len_s.end());
	output = len_s + prefix;
}


int main (int argc, char **argv){
	
	ros::init(argc, argv, "adhoc_sender1");
	ros::NodeHandle nh; 

	ros::ServiceServer sendPingService = nh.advertiseService("sendPing", sendPing);

	// get Parameters and print INFO
	nh.getParam("/sender/dst_car", dst_car);
	nh.getParam("/sender/mode", mode_i);
	nh.getParam("/sender/loop", loop);
	nh.getParam("/sender/strLen", strLen);
	nh.getParam("/sender/pos", pos);
	ROS_INFO("loop [%d], mode [%d], length/10 [%d], \n Dest: [%s], pos: [%s]", loop, mode_i, strLen, dst_car.c_str(), pos.c_str());
	
	Mode mode = static_cast<Mode>(mode_i);
	ping = (mode == PING);

	// dummy is 10Bytes, make longstring=strLen*10
	if (mode == STRING_SERIALIZE || mode == STRING_SERVICE){
		std::string dummy = "ABCDEFGHIJ";
		for(int k = 0; k<strLen; k++)
			longstring += dummy;
		std::string size;
		convertToPrefixString(longstring.length(), size);	
		std::cout << "Stringlength: "<< size << "Bytes\n";
	}

	// Generate Config-String
	std::ostringstream confStringStream;
	if (pos != "")
		confStringStream	<< "p" << pos;
	confStringStream	<< "m" << mode_i;
	//confStringStream	<< "s" << sleep;
	//if (sleep) 
	//	confStringStream 	<< "r" << rate;
	if (mode == STRING_SERIALIZE || mode == STRING_SERVICE) 
		confStringStream << "le" << strLen;

	// make absolute filepath
	std::string filepath = std::string("/home/pses/catkin_ws/src/adhoc_package/"+ confStringStream.str() +".csv");
	ROS_INFO("Filepath [%s]", filepath.c_str());

	// call set Filepath	
	ros::ServiceClient client = nh.serviceClient<adhoc_tests::FilenameService>("setFilename");
	adhoc_tests::FilenameService fnameService;
	fnameService.request.filename = filepath;
	ROS_INFO("before set filepath");
	if (client.call(fnameService))
		ROS_INFO("Filepath set with Service");
	else
		ROS_ERROR("Failed to call FilenameService");
	ROS_INFO("after set filepath");
/*
	IPERF
	PING6
	pECKEs1r(<30) messen !!! 
*/

	ros::spin();

	return 1;
}