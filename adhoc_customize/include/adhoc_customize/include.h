
// DO NOT CHANGE
#define FRAME_DATA_TYPE_CUSTOM 0xf0

//Begin Custom with 0xF1, 0xF2...
#define FRAME_DATA_TYPE_RECTANGLE 0xf1
#define FRAME_DATA_TYPE_STRING 0xf2
#define FRAME_DATA_TYPE_STUDENT 0xf3
#define FRAME_DATA_TYPE_TIME 0xf4
#define FRAME_DATA_TYPE_STRING_W_TIME 0xf5
#include "adhoc_customize/Rectangle.h"
#include "adhoc_customize/RecvTime.h"
#include "adhoc_customize/Student.h"
//#include "adhoc_messages/Square.h"
#include "adhoc_customize/StringWTime.h"

#include "std_msgs/String.h"
#include "std_msgs/Time.h"

#ifdef COMM
void publishCustomMessage(std::string payload, std::string topic, uint8_t data_type, std::string src_host){
	if (data_type == FRAME_DATA_TYPE_RECTANGLE){        
        ROS_INFO("FRAME_DATA_TYPE_RECTANGLE");
        adhoc_customize::Rectangle rect;
        desializeObject((unsigned char*) payload.data(), payload.length(), &rect);
        publishMessage(rect, topic);
    }else 
    if (data_type == FRAME_DATA_TYPE_STRING_W_TIME){
        ROS_INFO("FRAME_DATA_TYPE_STRING_W_TIME");
        adhoc_customize::StringWTime strWTime;
        desializeObject((unsigned char*) payload.data(), payload.length(), &strWTime);
        publishMessage(strWTime, topic);
    }else
    if (data_type == FRAME_DATA_TYPE_STUDENT){        
        ROS_INFO("FRAME_DATA_TYPE_STUDENT");
        adhoc_customize::Student stud;
        desializeObject((unsigned char*) payload.data(), payload.length(), &stud);
        publishMessage(stud, topic);
    }else
    if (data_type == FRAME_DATA_TYPE_TIME){        
        ROS_INFO("FRAME_DATA_TYPE_TIME");
        std_msgs::Time std_time;
        desializeObject((unsigned char*) payload.data(), payload.length(), &std_time);
        adhoc_customize::RecvTime recvTime;
        recvTime.time = std_time.data;
        recvTime.src_car=src_host;
        publishMessage(recvTime, topic);
    }else
    
    ROS_ERROR("UNKNOWN FRAME_DATA_TYPE");
}
#endif

//blabla