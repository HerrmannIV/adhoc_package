if (p->data_type_ == FRAME_DATA_TYPE_RECTANGLE){        
                    adhoc_messages::Rectangle rect;
                    desializeObject((unsigned char*) payload.data(), payload.length(), &rect);
                    publishMessage(rect, p->topic_);
                }