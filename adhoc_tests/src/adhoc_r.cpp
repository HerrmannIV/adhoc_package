#include "ros/ros.h"
#include "adhoc_customize/include.h"
#include "adhoc_tests/FilenameService.h"
#include <adhoc_tests/sendPing.h>

#include <iostream> 
#include <fstream>

ros::NodeHandle* nhp;
std::ofstream outFile;
int recvCounter = 0, linecounter = 0;

void answerCallback(const adhoc_customize::RecvTime::ConstPtr& recvTime);
void filenameCallback(const std_msgs::String::ConstPtr& msg);
bool setFilename(adhoc_tests::FilenameService::Request &req, adhoc_tests::FilenameService::Response &res);



int main (int argc, char **argv){	
	ros::init(argc, argv, "adhoc_timeRecv");
	ros::NodeHandle nh;
	nhp = &nh;
	ros::Subscriber sub_answer = nh.subscribe("t_answer", 1000, answerCallback); 
	ros::Subscriber sub_filename = nh.subscribe("t_filename", 1000, filenameCallback); 
	ros::ServiceServer fnameService = nh.advertiseService("setFilename", setFilename);
	//ros::spin();
	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::waitForShutdown();
	outFile.close();
	return 1;
}

void answerCallback(const adhoc_customize::RecvTime::ConstPtr& recvTime){
	ros::Time afterSend = ros::Time::now();
	float sendTimeSec = (afterSend - recvTime->time).toSec();
	ROS_INFO("%d.%d: RecvAnswer, Duration: [%f] sec", recvCounter, linecounter, sendTimeSec);
	
	//ignore first	
	if(recvCounter){
		outFile << sendTimeSec <<";";
		linecounter++;
	}

	//make lines 25 wide
	if(linecounter == 25){
		linecounter=0;
		outFile << "\n";
	}
	recvCounter++;

	// send newPing
	ros::NodeHandle nh;
	ros::ServiceClient pingClient = nh.serviceClient<adhoc_tests::sendPing>("sendPing");
	adhoc_tests::sendPing sendPing;
	if (pingClient.call(sendPing))
		ROS_INFO("sentPing");
	else
		ROS_ERROR("Error calling Ping service or loop is over");
}    

void filenameCallback(const std_msgs::String::ConstPtr& msg){
	outFile << "\n";
	outFile.close();
	recvCounter = 0; linecounter = 0;
	outFile.open (msg->data.c_str(), std::ios_base::app);
	ROS_INFO("Filepath: [%s]", msg->data.c_str());
}

bool setFilename(adhoc_tests::FilenameService::Request &req, adhoc_tests::FilenameService::Response &res){
	outFile << "\n";
	outFile.close();
	recvCounter = 0; linecounter = 0;
	outFile.open (req.filename.c_str(), std::ios_base::app);
	ROS_INFO("Filepath from Service: [%s]", req.filename.c_str());
	
	// request first ping
	ros::NodeHandle nh;
	ros::ServiceClient pingClient = nh.serviceClient<adhoc_tests::sendPing>("sendPing");
	adhoc_tests::sendPing sendPing;
	ROS_INFO("before first ping req");
	if (
		ros::service::call("sendPing", sendPing))
		//pingClient.call(sendPing))
		ROS_INFO("sent first Ping");
	else
		ROS_ERROR("Error calling first Ping service");
	ROS_INFO("after first ping req");
	return true;
}