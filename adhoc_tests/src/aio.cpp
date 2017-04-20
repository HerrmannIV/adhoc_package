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
	PING = 5,
	DATA = 1,
	SERVICE = 2
};

std::string size;
int timeoutCounter;
std::ofstream outFile;
int receivedFirst = 0, linecounter = 0, readingCounter=0;
State state, nextState;
bool received = false, finish = false, timeout = false, ping, found, empty, output;
ros::Time sentTime, startTime, finishTime;
ros::Duration span;

int loop, mode_i, strLen, rate, sleeps;
std::string dst_car, pos, longstring;

std_msgs::Time timeMsg;
adhoc_customize::StringWTime strWTime;
double timeoutTime;
double valArray[1000];
double sum = 0.0, avg, thresh;
double quart1, med, quart3, quart90, min, max, overx = 0.0, timeoutRate;
int j;
std::ifstream file;
Mode mode;
std::string filepath;

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
		if(output) valArray[readingCounter] = sendTimeSec;
		readingCounter++;
	}else
		receivedFirst=1;
	
	if (readingCounter == loop || loop == 1){
		finish = true;
		finishTime = ros::Time::now();
	}
	if(readingCounter)
		ROS_INFO("%3d: Recv valid Answer: [%f] sec", readingCounter, sendTimeSec);

	received=true;
 }

void makeLongstring(){
	std::string dummy = "ABCDEFGHIJ";
	for(int k = 0; k<strLen; k++)
		longstring += dummy;
	convertToPrefixString(longstring.length(), size);	
	std::cout << "Stringlength: "<< size << "Bytes\n";
	strWTime.data = longstring;
	}
void generateFilename(){
	// Generate Config-String
	std::ostringstream confStringStream;
	confStringStream	<< "m" << mode_i;
	//if (mode == DATA)
	//	confStringStream << "le" << strLen;
	// make absolute filepath and open file
	filepath = std::string("/home/pses/catkin_ws/src/adhoc_package/"+ confStringStream.str() +".csv");
	ROS_INFO("Filepath [%s]", filepath.c_str());
}

void initNode(){

}
int main (int argc, char **argv){
	ros::init(argc, argv, "adhoc_aio");
	ros::NodeHandle nh; 
	ros::Subscriber sub_answer = nh.subscribe("t_answer", 1000, answerCallback); 
	ros::AsyncSpinner spinner(4);
	spinner.start();

	// get Parameters and print INFO
	nh.getParam("/sender/dst_car", dst_car);
	nh.getParam("/sender/rate", rate);
	nh.getParam("/sender/sleep", sleeps);
	nh.getParam("/sender/mode", mode_i);
	nh.getParam("/sender/loop", loop);
	nh.getParam("/sender/strLen", strLen);
	nh.getParam("/sender/pos", pos);
	nh.getParam("/sender/thresh", thresh);
	nh.getParam("/sender/timeout", timeoutTime);
	nh.getParam("/sender/output", output);

	ROS_INFO("loop [%d], mode [%d], length/10 [%d], \n Dest: [%s], pos: [%s], timeouttime: [%f]", loop, mode_i, strLen, dst_car.c_str(), pos.c_str(), timeoutTime);
	
	mode = static_cast<Mode>(mode_i);
	ping = (mode == PING);

	if (mode == DATA) makeLongstring();

	
	generateFilename();
	ros::Rate loop_rate(rate);
	state = SEND;
	startTime = ros::Time::now();
	while(ros::ok()){
		if(finish)
			state = FINISH;
		switch (state){
		case SEND:
			ROS_INFO("send");
			sentTime = ros::Time::now();
			switch (mode){
				case PING:					
					timeMsg.data = sentTime;
					adhoc_communication::sendMessage(timeMsg, FRAME_DATA_TYPE_TIME, dst_car, "t_ping");
					break;
				case DATA:					
					strWTime.time = sentTime;
					adhoc_communication::sendMessage(strWTime, FRAME_DATA_TYPE_STRING_W_TIME, dst_car, "t_sswt");
					break;

			}
			if (loop==1) return 1;
			nextState = WAIT_FOR_ANSWER;
			break;

		case WAIT_FOR_ANSWER:
			//ROS_INFO("wait");
			span = ros::Time::now() - sentTime;
			timeout = (span.toSec() > timeoutTime);

			if (timeout || received){
				if (timeout) {
					timeoutCounter++;
					ROS_ERROR("TIMEOUT");
				}
				received = false;
				timeout = false;
				nextState = SEND;			
			}
			else{
				nextState = state;
			}
			break;

		case FINISH:
			ROS_INFO("Finish");
			if (output){
				// sort readings
				std::sort(valArray, valArray + loop);
				for(int i=0; i<loop; i++){
					sum += valArray[i];
				}
				ros::Duration overallTime = finishTime - startTime;
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
				ROS_INFO(" quart3[%f]; max [%f]; over: [%f]; timeouts: [%d];", quart3, max, overx, timeoutCounter);

				// output to file 			
				outFile.open (filepath.c_str(), std::ios_base::app);
				outFile << pos << ";";
				outFile << loop << ";";
				if (mode = DATA) outFile << size << ";";
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
				outFile << timeoutRate << ";";
				outFile << sum << ";";
				outFile << overallTime.toSec() << ";\n";


				outFile.close();
				ROS_INFO("Finished output");
			}
			return 1;

		default: 
			ROS_INFO("ILLEGAL STATE");

		}
		if (sleeps){
			ROS_INFO("sleeping");
		 loop_rate.sleep();
		}
		state = nextState;
	}
}

