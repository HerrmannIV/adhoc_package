#include "ros/ros.h"
#include "adhoc_customize/include.h"

std::ofstream outFile;
int recvCounter = 0, linecounter = 0;

void answerCallback(const adhoc_customize::RecvTime::ConstPtr& recvTime){
	ros::Time afterSend = ros::Time::now();
	float sendTimeSec = (afterSend - recvTime->time).toSec();
	ROS_INFO("%d.%d: RecvAnswer, Duration: [%f] sec", recvCounter, linecounter, sendTimeSec);
	
	if(recvCounter){
		outFile << sendTimeSec <<";";
		linecounter++;
	}
	if(linecounter == 25){
		linecounter=0;
		outFile << "\n";
	}
	recvCounter++;
}
void filenameCallback(const std_msgs::String::ConstPtr& msg){
	outfile.close();
	std::string fname = std::string("/home/pses/catkin_ws/src/adhoc_package/"+ msg->data.c_str() +".csv");
	outFile.open (fname.c_str(), std::ios_base::app);
	ROS_INFO("File changed to [%s]", fname.c_str());
}


int main (int argc, char **argv){	
	ros::init(argc, argv, "adhoc_timeRecv");
	ros::NodeHandle nh;
	ros::Subscriber sub_answer = nh.subscribe("t_answer", 1000, answerCallback); 
	ros::Subscriber sub_answer = nh.subscribe("t_filename", 1000, filenameCallback); 
	ros::spin();
	outFile.close();
	return 1;
}