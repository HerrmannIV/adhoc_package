#include "ros/ros.h"

#include "adhoc_customize/include.h"
#include "adhoc_communication/functions.h"

#include "adhoc_tests/FilenameService.h"
#include <adhoc_tests/sendPing.h>

#include <string> 
#include <iostream> 
#include <fstream>
#include <algorithm>


enum State{
	INIT,
	SEND,
	WAIT_FOR_ANSWER,
	FINISH
};

enum Mode{
	PING,
	STRING_SERIALIZE,
	STRING_SERVICE,
	RECT,
	STUD,
	PING_ALT
};
int timeoutCounter;
std::ofstream outFile;
int receivedFirst = 0, linecounter = 0, readingCounter=0;
State state, nextState;
bool received = false, finish = false, timeout = false, ping, found, empty ;
ros::Time sentTime;
ros::Duration span;
ros::Duration oneSec(1,0);

int loop, mode_i, strLen;
std::string dst_car, pos, longstring;

std_msgs::Time timeMsg;
double valArray[1000];
double sum = 0.0, avg, thresh;
double quart1, med, quart3, quart90, min, max, overx = 0.0, timeoutRate;
int j;
std::ifstream file;

void convertToPrefixString(int input, std::string &output){
	// convert int input to string
	std::ostringstream Stream;
    Stream << input;
	std::string len_s = Stream.str();
	
	// set prefix and crop amount
	std::string prefix = ""; int crop=0;
	if (input >=1000) { prefix = "k"; crop=3;}
	if (input >=1000000) { prefix = "M"; crop=6;}

	// crop and add prefix
	len_s.erase(len_s.end()-crop, len_s.end());
	output = len_s + prefix;
}

void answerCallback(const adhoc_customize::RecvTime::ConstPtr& recvTime){
	ros::Time afterSend = ros::Time::now();
	float sendTimeSec = (afterSend - recvTime->time).toSec();
	

	//ignore first	
	if(receivedFirst){
		valArray[readingCounter] = sendTimeSec;
		//outFile << sendTimeSec <<";";
		linecounter++;
		readingCounter++;
	}else{
		receivedFirst=1;
	}

	//make lines 25 wide
	if(linecounter == 25){
		linecounter=0;
		//outFile << "\n";
	}
	if (readingCounter == loop)
		finish = true;
	if(readingCounter){
		//std::cout << "\r        " << readingCounter;
		ROS_INFO("%d.%2d: Recv valid Answer: [%f] sec", readingCounter, linecounter, sendTimeSec);
	}
	received=true;
 }

int main (int argc, char **argv){

	ros::init(argc, argv, "adhoc_aio");
	ros::NodeHandle nh; 
	ros::Subscriber sub_answer = nh.subscribe("t_answer", 1000, answerCallback); 
	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::Rate rate(100);
	// get Parameters and print INFO
	nh.getParam("/sender/dst_car", dst_car);
	nh.getParam("/sender/mode", mode_i);
	nh.getParam("/sender/loop", loop);
	nh.getParam("/sender/strLen", strLen);
	nh.getParam("/sender/pos", pos);
	nh.getParam("/sender/thresh", thresh);
	ROS_INFO("loop [%d], mode [%d], length/10 [%d], \n Dest: [%s], pos: [%s]", loop, mode_i, strLen, dst_car.c_str(), pos.c_str());
	
	Mode mode = static_cast<Mode>(mode_i);
	ping = (mode == PING);

	// dummy is 10Bytes, make longstring=strLen*10
	if (mode == STRING_SERIALIZE || mode == STRING_SERVICE){
		std::string dummy = "ABCDEFGHIJ";
		for(int k = 0; k<strLen; k++)
			longstring += dummy;
		std::string size;
		convertToPrefixString(longstring.length(), size);	
		std::cout << "Stringlength: "<< size << "Bytes\n";
	}

	// Generate Config-String
	std::ostringstream confStringStream;
	//if (pos != "")
		//confStringStream	<< "p" << pos;
	confStringStream	<< "m" << mode_i;
	if (mode == STRING_SERIALIZE || mode == STRING_SERVICE) 
		confStringStream << "le" << strLen;

	// make absolute filepath and open file
	std::string filepath = std::string("/home/pses/catkin_ws/src/adhoc_package/"+ confStringStream.str() +".csv");
	ROS_INFO("Filepath [%s]", filepath.c_str());
	state = SEND;

	while(ros::ok()){

		if(finish)
			state = FINISH;

		switch (state){
		case SEND:
			sentTime = ros::Time::now();
			timeMsg.data = sentTime;
			adhoc_communication::sendMessage(timeMsg, FRAME_DATA_TYPE_TIME, dst_car, "t_ping");
			nextState = WAIT_FOR_ANSWER;
			break;
		case WAIT_FOR_ANSWER:
			
			span = ros::Time::now() - sentTime;
			
			//timeout = (span.toSec() > 1.0);
			//if (timeout || received){

			if ((span.toSec() > 1.0) || received){

				/*
				if (timeout) {
					timeoutCounter++;
					ROS_ERROR("TIMEOUT");
				}*/
				received = false;
				timeout = false;
				nextState = SEND;			
			}
			else{
				nextState = state;
				//rate.sleep();
			}
			break;
		case FINISH:
			ROS_INFO("Finish");

			// sort readings
			std::sort(valArray, valArray + loop);
			for(int i=0; i<loop; i++){
				sum += valArray[i];
			}
			
			// calc stats
			avg = sum/loop;
			min = valArray[0];
			quart1 = valArray[loop/4];
			med = valArray[loop/2];
			quart3 = valArray[(3 * loop)/4];
			quart90 = valArray[(9 * loop)/10];
			max = valArray[loop-1];
			timeoutRate = (double)timeoutCounter/(double)loop;
			for(j=0; j<loop && !found; ++j){
				//std::cout << j << ":[" << valArray[j] << "]" << std::endl;
				if (valArray[j+1] >= thresh)
					found=true;
			}
			//std::cout << j ;
			if (found) overx = (double)(loop - j) / (double)loop;
			ROS_INFO("avg [%f]; min [%f]; quart1 [%f]; med[%f];", avg, min, quart1, med);
			ROS_INFO(" quart3[%f]; max [%f]; over: [%f]; timeout: [%d];", quart3, max, overx, timeoutCounter);

			// output to file 			
			outFile.open (filepath.c_str(), std::ios_base::app);
			outFile << pos << ";";
			outFile << loop << ";";
			outFile << avg << ";";
			outFile << min << ";";
			outFile << quart1 << ";";
			outFile << med << ";";
			outFile << quart3 << ";";
			outFile << quart90 << ";";
			outFile << max << ";";
			outFile << thresh << ";";
			outFile << overx << ";";
			outFile << timeoutCounter << ";";
			outFile << timeoutRate<< ";\n";

			outFile.close();
			ROS_INFO("Finished output");

			return 1;

		default: 
			ROS_INFO("ILLEGAL STATE");

		}
		state = nextState;
	}
}

