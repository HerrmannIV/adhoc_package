
// DO NOT CHANGE
#define FRAME_DATA_TYPE_CUSTOM 0xf0

//Begin Custom with 0xF1, 0xF2...
#define FRAME_DATA_TYPE_RECTANGLE 0xf1
#define FRAME_DATA_TYPE_STRING 0xf2

#include "adhoc_customize/Rectangle.h"
//#include "adhoc_messages/Square.h"

#include "std_msgs/String.h"

#ifdef COMM
void publishCustomMessage(std::string payload, std::string topic, uint8_t data_type){
	if (data_type == FRAME_DATA_TYPE_RECTANGLE){        
        adhoc_customize::Rectangle rect;
        desializeObject((unsigned char*) payload.data(), payload.length(), &rect);
        publishMessage(rect, topic);
    }

	if (data_type == FRAME_DATA_TYPE_STRING){        
        std_msgs::String str;
        desializeObject((unsigned char*) payload.data(), payload.length(), &str);
        publishMessage(str, topic);
    }

}
#endif