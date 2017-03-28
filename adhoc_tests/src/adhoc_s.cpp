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
std::ofstream outFile;
adhoc_customize::Rectangle rectangle;
std::string longstring;


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
	int rate, loop, mode_i,strLen, sleep;
	std::string dst_car, filename;
	nh.getParam("/sender/sleep", sleep);
	nh.getParam("/sender/dst_car", dst_car);
	nh.getParam("/sender/rate", rate);
	nh.getParam("/sender/mode", mode_i);
	nh.getParam("/sender/loop", loop);
	nh.getParam("/sender/strLen", strLen);
	nh.getParam("/sender/filename", filename);
	ROS_INFO("loop [%d]; mode [%d]: rate [%d]; length/10 [%d], Dest: [%s], sleep:[%d]", loop, mode_i, rate, strLen, dst_car.c_str(), sleep);
	//std::string fname = std::string("/home/pses/catkin_ws/src/adhoc_package/" + filename + ".csv");
	
	
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
	// if ping, save Output to file

	std::ostringstream confStringStream;
	confStringStream	//<< "loop: " << loop
						<< "m" << mode_i
						<< "r" << rate
						//<< "; length/10: " << strLen
						//<< "; Dest: " << dst_car
						<< "s" << sleep;

	std::string fname = std::string("/home/pses/catkin_ws/src/adhoc_package/"+ confStringStream.str() +".csv");
	std::ifstream inFile(fname.c_str());
	std::string inputLine;
	if (inFile.is_open()){
		getline(inFile, inputLine);
		inFile.close();
		std::cout << "|" << inputLine << "|\n";
		std::cout << "|" << confStringStream.str() << "|\n";
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
	outFile.open (fname.c_str(), std::ios_base::app);
	int i = 0;
	ros::Time begin = ros::Time::now();
	while(ros::ok() && i<loop){
		i++;
		if(mode==PING){
			// setup ROS-stuff
			ros::ServiceClient client = nh.serviceClient<adhoc_communication::SendString>("adhoc_communication/send_string");
	    	adhoc_communication::SendString srv;
	    	// setup message, data is topic of returned ping
	    	std::stringstream ss; ss << i;
	    	std::string returnTopic = "top_ping" + ss.str();
		    srv.request.topic = "t_ping";
		    srv.request.data = returnTopic;
		    srv.request.dst_robot = dst_car;
		    
		    //PING
		    beforePing = ros::Time::now();
		    if (!client.call(srv)) ROS_ERROR("Failed to call PING/STRING-service");
			ros::topic::waitForMessage<adhoc_communication::RecvString>(returnTopic , ros::Duration(2));
			ros::Time afterPing = ros::Time::now();

			// calc Latency and output (to file)
			ros::Duration pingTime = afterPing - beforePing;
			float pingTimeSec = pingTime.toSec();
			if (pingTimeSec > 1.5)
				ROS_INFO("Ping Timeout: [%f] sec", pingTimeSec);
			else
				outFile << pingTimeSec <<";";
			std::cout << pingTimeSec << "\n";

		}else if(mode==STRING_SERIALIZE){
			// send String with my own Serialization method
			std_msgs::String str;
			str.data = longstring;
			adhoc_communication::sendMessage(str, FRAME_DATA_TYPE_STRING, dst_car, "t_String");
		}else if(mode==STRING_SERVICE){
			// send String with native sendString Service 
			ros::ServiceClient client = nh.serviceClient<adhoc_communication::SendString>("adhoc_communication/send_string");
	    	adhoc_communication::SendString srv;
	    	// fill Service-Fields
		    srv.request.topic = "t_string";
		    srv.request.data = longstring;
		    srv.request.dst_robot = dst_car;
		    		    
		    // call Service
		    if (!client.call(srv)) ROS_ERROR("Failed to call PING/STRING-service");

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

	ros::Time end = ros::Time::now();
	ros::Duration dur = end-begin;
	ROS_INFO("duration: %f sec", dur.toSec());

	outFile << "\n";
	outFile.close();

	return 1;
}