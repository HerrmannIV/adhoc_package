#include "ros/ros.h"
#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"
#include "adhoc_communication/SendString.h"
#include "adhoc_communication/RecvString.h"

#include <iostream> 

enum Mode{
	PING,
	STRING_SERIALIZE,
	STRING_SERVICE,
	RECT,
	STUD
};

ros::Time beforePing;

void convertToPrefixString(int input, std::string &output){
	// convert input to string
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

void pingCallback(const adhoc_communication::RecvString::ConstPtr& msg){
	ros::Time afterPing = ros::Time::now();
	ros::Duration ping = afterPing - beforePing;
	ROS_INFO("Ping duration: [%f] sec", ping.toSec());
}

int main (int argc, char **argv){
	
	ros::init(argc, argv, "adhoc_sender1");
	ros::NodeHandle nh;

	// get Parameters and print INFO
	int rate, loop, mode_i,strLen;
	std::string dst_robot;	
	nh.getParam("/sender/dst_robot", dst_robot);
	nh.getParam("/sender/rate", rate);
	nh.getParam("/sender/mode", mode_i);
	nh.getParam("/sender/loop", loop);
	nh.getParam("/sender/strLen", strLen);
	ROS_INFO("loop [%d]; mode [%d]: rate [%d]; length/10 [%d], Dest: [%s]", loop, mode_i, rate, strLen, dst_robot.c_str());
	Mode mode = static_cast<Mode>(mode_i);
	//ros::Rate loop_rate(rate);
	adhoc_customize::Rectangle rectangle;
	int i = 0;

	// dummy is 10Bytes, make longstring
	std::string longstring;
	std::string dummy = "ABCDEFGHIJ";
	for(int k = 0; k<strLen; k++)
		longstring += dummy;
	std::string size;
	convertToPrefixString(longstring.length(), size);	
	std::cout << "Stringlength: "<< size << "Bytes\n";

	ros::Subscriber sub_ping = nh.subscribe("t_ping", 1000, pingCallback);  





	ros::Time begin = ros::Time::now();

	while(ros::ok() && i<loop){
		i++;
		if(mode==STRING_SERIALIZE){
			// send String with my own Serialization method
			std_msgs::String str;
			str.data = longstring;
			adhoc_communication::sendMessage(str, FRAME_DATA_TYPE_STRING, dst_robot, "t_String");
		}else if(mode==STRING_SERVICE || mode==PING){
			bool ping = (mode == PING);
			// send String with native sendString Service or empty string as Ping
			ros::ServiceClient client = nh.serviceClient<adhoc_communication::SendString>("adhoc_communication/send_string");
	    	adhoc_communication::SendString srv;
	    	
	    	// fill Service-Fields
		    srv.request.topic = ping ? "t_ping": "t_string";
		    srv.request.data = ping ? "" : longstring;
		    srv.request.dst_robot = dst_robot;
		    		    
		    // call Service
		    if(ping) ros::Time beforePing = ros::Time::now();

		    if (client.call(srv)){
		        //ROS_INFO("Response.status: %d", srv.response.status);
		    }else{
		        ROS_ERROR("Failed to call service");
		    }

		}else if(mode==RECT){
			// send Rectangle
			rectangle.length = i;
			rectangle.width = i;
			adhoc_communication::sendMessage(rectangle, FRAME_DATA_TYPE_RECTANGLE, dst_robot, "t_rectangle");
		}else if(mode==STUD){
			adhoc_customize::Student stud;
			stud.name = "Rau";
			stud.vorname = "Kai";			
			stud.immatrikuliert = true;
			stud.matnr = 123;
			adhoc_communication::sendMessage(stud, FRAME_DATA_TYPE_STUDENT, dst_robot, "t_stud");
		}	    

		//loop_rate.sleep();
	}
	ros::Time end = ros::Time::now();
	ros::Duration dur = end-begin;
	ROS_INFO("duration: %f sec", dur.toSec());

	return 1;
}

