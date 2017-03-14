#include "ros/ros.h"
#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"
#include "adhoc_communication/SendString.h"


int main (int argc, char **argv){
	
	ros::init(argc, argv, "adhoc_sender1");
	ros::NodeHandle nh;
	int rate;

	nh.getParam("/rate", rate);
	ROS_INFO("sending at rate %d Hz", rate);
	ros::Rate loop_rate(rate);
	int count = 0;

	adhoc_customize::Rectangle rectangle;
	int i = 0;
	rectangle.length = i;
	rectangle.width = i;

	ros::Time begin = ros::Time::now();
	
	std::string longstring; 
	std::string dummy = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";//10 Byte
	for(int k = 0; k<100; k++)
		longstring += dummy;


	while(ros::ok()){
		std::string dst_robot;
		nh.getParam("/dest_robot", dst_robot);
		//adhoc_communication::sendMessage(rectangle, FRAME_DATA_TYPE_RECTANGLE, dst_robot, "t_rectangle");
		if(false){
			std_msgs::String str;
			str.data = longstring;
			adhoc_communication::sendMessage(str, FRAME_DATA_TYPE_STRING, dst_robot, "t_String");
		}else{		
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
		}
		

	    loop_rate.sleep();

		if (i<=3) i++;
		else {
			ros::Time end = ros::Time::now();
			ros::Duration dur = end-begin;
			ROS_INFO("dur: %f",dur.toSec());

			return 1; //i = 65;
		}
		rectangle.length = i;
		rectangle.width = i;

	}
	return 1;
}

