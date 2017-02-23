#include "ros/ros.h"
//#include "std_msgs/String.h"

#include "adhoc_messages/Rectangle.h"
#include "adhoc_messages/include.h"
//#include <adhoc_communication/AnyMsg.h>
//#include <adhoc_communication/AnyMsgWTopic.h>
#include "adhoc_communication/SendAnything.h"
#include "adhoc_communication/functions.h"

//#define MSG_TYPE_RECT 0x01

int main (int argc, char **argv){
	
	ros::init(argc, argv, "adhoc_sender1");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(1);
	int count = 0;

	adhoc_messages::Rectangle rectangle;
	int i = 65;
	rectangle.length = i;
	rectangle.width = i;


	while(ros::ok()){
		std::string dst_robot;
		nh.getParam("/dest_robot", dst_robot);
		std::string msg = getSerializedMessage(rectangle);
		adhoc_communication::sendMessage(rectangle, FRAME_DATA_TYPE_RECTANGLE, dst_robot, "t_rectangle");

		loop_rate.sleep();

		if (i<=89) i++;
		else i = 65;

		rectangle.length = i;
		rectangle.width = i;		
	}
	return 1;
}

