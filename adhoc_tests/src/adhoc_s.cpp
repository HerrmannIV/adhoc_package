#include "ros/ros.h"
#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"
#include "adhoc_communication/SendString.h"

int main (int argc, char **argv){
	
	ros::init(argc, argv, "adhoc_sender1");
	ros::NodeHandle nh;

	// get Parameters
	int rate, loop, mode,strLen;
	std::string dst_robot;
	
	nh.getParam("/sender/dest_robot", dst_robot);
	nh.getParam("/sender/rate", rate);
	nh.getParam("/sender/mode", mode);
	nh.getParam("/sender/loop", loop);
	nh.getParam("/sender/strLen", strLen);

	ROS_INFO("sending [%d] times in mode [%d] to [%s] at loop_rate [%d] Hz", loop, mode, dst_robot.c_str(), rate);
	ros::Rate loop_rate(rate);

	adhoc_customize::Rectangle rectangle;
	int i = 0;

	

	// dummy is 10Bytes, make longstring 100kB
	std::string longstring;
	std::string dummy = "XXXXXXXXXX";
	for(int k = 0; k<strLen; k++)
		longstring += dummy;
	std::cout << "Stringlength: ["<< longstring.length() << "] Bytes\n";
	ros::Time begin = ros::Time::now();

	while(ros::ok() && i<loop){

		if(mode==1){
			// send String with my own Serialization method
			std_msgs::String str;
			str.data = longstring;
			adhoc_communication::sendMessage(str, FRAME_DATA_TYPE_STRING, dst_robot, "t_String");
		}else if(mode==2){
			// send String with native sendString Service
			ros::ServiceClient client = nh.serviceClient<adhoc_communication::SendString>("adhoc_communication/send_string");
	    	adhoc_communication::SendString srv;
	    
	    	// fill Service-Fields
		    srv.request.topic = "t_string";
		    srv.request.dst_robot = dst_robot;
		    srv.request.data = longstring;

		    // call Service
		    if (client.call(srv)){
		        ROS_INFO("Response.status: %d", srv.response.status);
		    }else{
		        ROS_ERROR("Failed to call service");
		    } 
		}else if(mode==3){
			// send Rectangle
			rectangle.length = i;
			rectangle.width = i;
			adhoc_communication::sendMessage(rectangle, FRAME_DATA_TYPE_RECTANGLE, dst_robot, "t_rectangle");
		}	    

		i++;
		rectangle.length = i;
		rectangle.width = i;
		loop_rate.sleep();
	}
	ros::Time end = ros::Time::now();
	ros::Duration dur = end-begin;
	ROS_INFO("duration: %f sec", dur.toSec());

	return 1;
}

