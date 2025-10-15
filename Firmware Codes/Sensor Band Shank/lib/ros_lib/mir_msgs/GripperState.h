#ifndef _ROS_mir_msgs_GripperState_h
#define _ROS_mir_msgs_GripperState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class GripperState : public ros::Msg
  {
    public:
      typedef const char* _state_string_type;
      _state_string_type state_string;
      typedef uint8_t _state_type;
      _state_type state;
      typedef bool _closed_type;
      _closed_type closed;

    GripperState():
      state_string(""),
      state(0),
      closed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_state_string = strlen(this->state_string);
      varToArr(outbuffer + offset, length_state_string);
      offset += 4;
      memcpy(outbuffer + offset, this->state_string, length_state_string);
      offset += length_state_string;
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      union {
        bool real;
        uint8_t base;
      } u_closed;
      u_closed.real = this->closed;
      *(outbuffer + offset + 0) = (u_closed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->closed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_state_string;
      arrToVar(length_state_string, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state_string; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state_string-1]=0;
      this->state_string = (char *)(inbuffer + offset-1);
      offset += length_state_string;
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
      union {
        bool real;
        uint8_t base;
      } u_closed;
      u_closed.base = 0;
      u_closed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->closed = u_closed.real;
      offset += sizeof(this->closed);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/GripperState"; };
    virtual const char * getMD5() override { return "163f067f3329a2206fb2ea4c82fe8782"; };

  };

}
#endif
