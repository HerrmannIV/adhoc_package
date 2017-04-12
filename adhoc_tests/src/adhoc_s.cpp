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
int recvCounter = 0, linecounter = 0;


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


	// get Parameters and print INFO
	int rate, loop, mode_i, strLen, sleep;
	std::string dst_car, pos;
	nh.getParam("/sender/sleep", sleep);
	nh.getParam("/sender/dst_car", dst_car);
	if (sleep)
		nh.getParam("/sender/rate", rate);
	else 
		rate = 2;
	nh.getParam("/sender/mode", mode_i);
	nh.getParam("/sender/loop", loop);
	nh.getParam("/sender/strLen", strLen);
	nh.getParam("/sender/pos", pos);
	ROS_INFO("loop [%d]; mode [%d]: rate [%d]; length/10 [%d], Dest: [%s], sleep:[%d], pos: [%s]", loop, mode_i, rate, strLen, dst_car.c_str(), sleep, pos.c_str());
	
	Mode mode = static_cast<Mode>(mode_i);
	ros::Rate loop_rate(rate);
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

	// Generate Filename from Config
	std::ostringstream confStringStream;
	if (pos != "")
		confStringStream	<< "p" << pos;
	confStringStream	<< "m" << mode_i;
	confStringStream	<< "s" << sleep;
	if (sleep) 
		confStringStream 	<< "r" << rate;
	if (mode == STRING_SERIALIZE || mode == STRING_SERVICE) 
		confStringStream << "le" << strLen;

	
	std::string filepath = std::string("/home/pses/catkin_ws/src/adhoc_package/"+ confStringStream.str() +".csv");
	ROS_INFO("Filepath [%s]", filepath.c_str());
	
	/*
	ros::Rate pubRate(1);
	ros::Publisher confPub = nh.advertise<std_msgs::String>("t_filename", 1000, false);
	for(int k=0; k<10; k++){
		confPub.publish(filepath);
		ros::spinOnce();
		pubRate.sleep();
	}
	*/

	/*
	while (confPub.getNumSubscribers()==0){
		ROS_INFO("bla");
		std::cout << "."; // << confPub.getNumSubscribers();
	}
	*/

	ros::ServiceClient client = nh.serviceClient<adhoc_tests::FilenameService>("setFilename");
	adhoc_tests::FilenameService fnameService;
	fnameService.request.filename = filepath;
	if (client.call(fnameService))
		ROS_INFO("Published Filepath");
	else
		ROS_ERROR("Failed to call FilenameService");
/*

	IPERF
	PING6
	pECKEs1r(<30) messen !!! 



*/


	int i = 0;

	while(ros::ok() && i<loop){
		if (!i) ROS_INFO("First sending");
		i++;
		if(mode==PING_ALT){
			std_msgs::Time timeMsg;
			timeMsg.data = ros::Time::now();
			adhoc_communication::sendMessage(timeMsg, FRAME_DATA_TYPE_TIME, dst_car, "t_ping");
		}else if(mode==STRING_SERIALIZE){
			// send String with my own Serialization method
			adhoc_customize::StringWTime strWTime;
			strWTime.data = longstring;
			strWTime.time = ros::Time::now();
			adhoc_communication::sendMessage(strWTime, FRAME_DATA_TYPE_STRING_W_TIME, dst_car, "t_sswt");
		}else if(mode==STRING_SERVICE){
			// send String with native sendString Service 
			ros::ServiceClient client = nh.serviceClient<adhoc_communication::SendString>("adhoc_communication/send_string");
	    	adhoc_communication::SendString srv;
	    	// fill Service-Fields
		    srv.request.topic = "t_stringService";
		    srv.request.data = longstring;
		    srv.request.dst_robot = dst_car;
		    		    
		    // call Service
		    if (!client.call(srv)) ROS_ERROR("Failed to call STRING-service");

		}else if(mode==RECT){
			// send Rectangle
			rectangle.length = i;
			rectangle.width = i;
			adhoc_communication::sendMessage(rectangle, FRAME_DATA_TYPE_RECTANGLE, dst_car, "t_rectangle");

		}else if(mode==STUD){
			adhoc_customize::Student stud;
			stud.name = "Rau";
			stud.vorname = "Kai";			
			stud.immatrikuliert = true;
			stud.matnr = 123;
			adhoc_communication::sendMessage(stud, FRAME_DATA_TYPE_STUDENT, dst_car, "t_stud");
		}
		if (sleep) loop_rate.sleep();
	}	
	return 1;
}