#include "ros/ros.h"
#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"
#include "adhoc_communication/SendString.h"

int main (int argc, char **argv){
	
	ros::init(argc, argv, "adhoc_sender1");
	ros::NodeHandle nh;

	// get Parameters
	int rate, loop, mode;
	std::string dst_robot;
	
	nh.getParam("/sender/dest_robot", dst_robot);
	nh.getParam("/sender/rate", rate);
	nh.getParam("/sender/mode", mode);
	nh.getParam("/sender/loop", loop);


	ROS_INFO("sending [%d] times in mode [%d] to [%s] at loop_rate [%d] Hz", loop, mode, dst_robot.c_str(), rate);
	ros::Rate loop_rate(rate);

	adhoc_customize::Rectangle rectangle;
	int i = 0;

	std::string longstring; 

	// dummy is 1000Bytes
	std::string dummy = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";//10 Byte
	for(int k = 0; k<100; k++)
		longstring += dummy;

	ros::Time begin = ros::Time::now();

	while(ros::ok()){
		if(mode==1){
			std_msgs::String str;
			str.data = longstring;
			adhoc_communication::sendMessage(str, FRAME_DATA_TYPE_STRING, dst_robot, "t_String");
		}else if(mode==2){		
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
			rectangle.length = i;
			rectangle.width = i;
			adhoc_communication::sendMessage(rectangle, FRAME_DATA_TYPE_RECTANGLE, dst_robot, "t_rectangle");
		}	    

		if (i<loop) 
			i++;
		else {
			ros::Time end = ros::Time::now();
			ros::Duration dur = end-begin;
			ROS_INFO("duration: %f sec", dur.toSec());

			return 1; //i = 65;
		}
		rectangle.length = i;
		rectangle.width = i;
		loop_rate.sleep();
	}
	return 1;
}

