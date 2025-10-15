#ifndef _ROS_mir_msgs_Events_h
#define _ROS_mir_msgs_Events_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mir_msgs/Event.h"

namespace mir_msgs
{

  class Events : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t events_length;
      typedef mir_msgs::Event _events_type;
      _events_type st_events;
      _events_type * events;

    Events():
      header(),
      events_length(0), st_events(), events(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->events_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->events_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->events_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->events_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->events_length);
      for( uint32_t i = 0; i < events_length; i++){
      offset += this->events[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t events_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      events_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      events_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      events_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->events_length);
      if(events_lengthT > events_length)
        this->events = (mir_msgs::Event*)realloc(this->events, events_lengthT * sizeof(mir_msgs::Event));
      events_length = events_lengthT;
      for( uint32_t i = 0; i < events_length; i++){
      offset += this->st_events.deserialize(inbuffer + offset);
        memcpy( &(this->events[i]), &(this->st_events), sizeof(mir_msgs::Event));
      }
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/Events"; };
    virtual const char * getMD5() override { return "7bc39c0a4512602f564b326a79461af0"; };

  };

}
#endif
