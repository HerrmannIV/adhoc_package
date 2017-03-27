#include "ros/ros.h"
#include "ros/topic.h"
#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"
#include "adhoc_communication/SendString.h"
#include "adhoc_communication/RecvString.h"

#include <iostream> 
#include <fstream>
#include <string> 


enum Mode{
	PING,
	STRING_SERIALIZE,
	STRING_SERVICE,
	RECT,
	STUD
};

ros::Time beforePing;
bool recvPing, ping;
std::ofstream myfile;

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
	recvPing=true;
}

int main (int argc, char **argv){
	
	ros::init(argc, argv, "adhoc_sender1");
	ros::NodeHandle nh;

	// get Parameters and print INFO
	int rate, loop, mode_i,strLen;
	std::string dst_car, filename;	
	nh.getParam("/sender/dst_car", dst_car);
	nh.getParam("/sender/rate", rate);
	nh.getParam("/sender/mode", mode_i);
	nh.getParam("/sender/loop", loop);
	nh.getParam("/sender/strLen", strLen);
	nh.getParam("/sender/filename", filename);
	ROS_INFO("loop [%d]; mode [%d]: rate [%d]; length/10 [%d], Dest: [%s]", loop, mode_i, rate, strLen, dst_car.c_str());
	Mode mode = static_cast<Mode>(mode_i);
	ros::Rate loop_rate(rate);
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

	//ros::Subscriber sub_ping = nh.subscribe("t_ping", 1000, pingCallback);  
	ping = (mode == PING);
	if(ping){
		std::string fname = std::string("/home/pses/catkin_ws/src/adhoc_package/" + filename + ".csv");
  		myfile.open (fname.c_str());//"/home/pses/" + filename + ".csv");
	}


	ros::Time begin = ros::Time::now();

	while(ros::ok() && i<loop){
		i++;
		if(mode==STRING_SERIALIZE){
			// send String with my own Serialization method
			std_msgs::String str;
			str.data = longstring;
			adhoc_communication::sendMessage(str, FRAME_DATA_TYPE_STRING, dst_car, "t_String");
		}else if(mode==STRING_SERVICE || ping){
			// send String with native sendString Service or empty string as Ping
			ros::ServiceClient client = nh.serviceClient<adhoc_communication::SendString>("adhoc_communication/send_string");
	    	adhoc_communication::SendString srv;
	    	std::stringstream ss;
	    	ss << i;
	    	std::string returnTopic = "top_ping" + ss.str();
	    	// fill Service-Fields
		    srv.request.topic = ping ? "t_ping" : "t_string";
		    srv.request.data = ping ? returnTopic : longstring;
		    srv.request.dst_robot = dst_car;
		    		    
		    // call Service
		    if(ping) {
		    	beforePing = ros::Time::now();
		    	//ROS_INFO("BeforeX: [%f] sec", beforePing.toSec());
		    }

		    if (client.call(srv)){
		        //ROS_INFO("Response.status: %d", srv.response.status);
		    }else{
		        ROS_ERROR("Failed to call PING/STRING-service");
		    }
		    if (ping){
			ros::topic::waitForMessage<adhoc_communication::RecvString>(returnTopic , ros::Duration(2));
			ros::Time afterPing = ros::Time::now();
			ros::Duration pingTime = afterPing - beforePing;
			float pingTimeSec = pingTime.toSec();
			if (pingTimeSec > 1.5)
				ROS_INFO("Ping Timeout: [%f] sec", pingTimeSec);
			else
				myfile << pingTimeSec <<"\n";

		}

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
		//if (ping) loop_rate.sleep();	    
	}	

	ros::Time end = ros::Time::now();
	ros::Duration dur = end-begin;
	ROS_INFO("duration: %f sec", dur.toSec());

	myfile.close();

	return 1;
}