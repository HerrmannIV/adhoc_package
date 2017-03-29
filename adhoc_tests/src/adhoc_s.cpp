#include "ros/ros.h"
#include "ros/topic.h"
#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"
#include "adhoc_communication/SendString.h"
#include "adhoc_communication/RecvString.h"

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
bool ping, received;
std::ofstream outFile;
adhoc_customize::Rectangle rectangle;
std::string longstring;
double pingTimeD;
int recvCounter = 0;


void convertToPrefixString(int input, std::string &output){
	// convert int input to string
	std::ostringstream Stream;
    Stream << input;
	std::string len_s = Stream.str();

	std::string prefix = ""; int crop=0;

	// set prefix and crop amount
	if (input >=1000) { prefix = "k"; crop=3;}
	if (input >=1000000) { prefix = "M"; crop=6;}

	// crop
	len_s.erase(len_s.end()-crop, len_s.end());
	output = len_s + prefix;
}

void pingCallback(const adhoc_customize::RecvTime::ConstPtr& recvTime){
	if(recvCounter){
		ros::Time afterPing = ros::Time::now();
		float pingTimeSec = (afterPing - recvTime->time).toSec();
		ROS_INFO("Ping duration: [%f] sec", pingTimeSec);
		outFile << pingTimeSec <<";";
	}
	recvCounter++;

}
void sendCallback(const adhoc_customize::RecvTime::ConstPtr& recvTime){
	if(recvCounter){
		ros::Time afterSend = ros::Time::now();
		float sendTimeSec = (afterSend - recvTime->time).toSec();
		ROS_INFO("RecvStrAns, Send duration: [%f] sec", sendTimeSec);
		outFile << sendTimeSec <<";";
	}
	recvCounter++;
}

int main (int argc, char **argv){
	
	ros::init(argc, argv, "adhoc_sender1");
	ros::NodeHandle nh;
	ros::Subscriber sub_ping = nh.subscribe("t_pingr", 1000, pingCallback);  
	ros::Subscriber sub_string = nh.subscribe("t_str", 1000, sendCallback);  
	ros::AsyncSpinner  spinner(1);
	spinner.start();
	// get Parameters and print INFO
	int rate, loop, mode_i, strLen, sleep;
	std::string dst_car;
	nh.getParam("/sender/sleep", sleep);
	nh.getParam("/sender/dst_car", dst_car);
	if (sleep)
		nh.getParam("/sender/rate", rate);
	else 
		rate = 2;
	nh.getParam("/sender/mode", mode_i);
	nh.getParam("/sender/loop", loop);
	nh.getParam("/sender/strLen", strLen);
	ROS_INFO("loop [%d]; mode [%d]: rate [%d]; length/10 [%d], Dest: [%s], sleep:[%d]", loop, mode_i, rate, strLen, dst_car.c_str(), sleep);
	
	Mode mode = static_cast<Mode>(mode_i);
	ros::Rate loop_rate(rate);
	ping = (mode == PING);

	// dummy is 10Bytes, make longstring
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
	confStringStream	<< "m" << mode_i;
	confStringStream	<< "s" << sleep;
	if (sleep) 
		confStringStream 	<< "r" << rate;
	if (mode == STRING_SERIALIZE || mode == STRING_SERVICE) 
		confStringStream << "le" << strLen;
	
	/*
	std::ifstream inFile(fname.c_str());
	std::string inputLine;
	if (inFile.is_open()){
		getline(inFile, inputLine);
		inFile.close();
		//std::cout << "|" << inputLine << "|\n";
		//std::cout << "|" << confStringStream.str() << "|\n";
		if (confStringStream.str().compare(inputLine) == 0)
			ROS_INFO("OutputFile same Config");
		else{ 
			return 0;
		}
	}else{
		ROS_INFO("Not open");
		outFile.open (fname.c_str());
		outFile << confStringStream.str() << "\n";
		outFile.close();		
	}
	*/
	ROS_INFO("%s",confStringStream.str().c_str());
	std::string fname = std::string("/home/pses/catkin_ws/src/adhoc_package/"+ confStringStream.str() +".csv");
	outFile.open (fname.c_str(), std::ios_base::app);
	int i = 0;

	//ros::Time begin = ros::Time::now();
	while(ros::ok() && i<loop){
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
			adhoc_communication::sendMessage(strWTime, FRAME_DATA_TYPE_STRING_W_TIME, dst_car, "t_stringSerializedWTime");
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
	/*
	ros::Time end = ros::Time::now();
	ros::Duration dur = end-begin;
	ROS_INFO("duration: %f sec", dur.toSec());
	*/
	while(recvCounter<=loop){
		ROS_INFO("COUNTER: [%d]", recvCounter);
		loop_rate.sleep();
	}
	outFile << "\n";
	outFile.close();
	return 1;
}