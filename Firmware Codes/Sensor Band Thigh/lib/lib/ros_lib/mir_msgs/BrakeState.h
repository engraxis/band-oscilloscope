#ifndef _ROS_mir_msgs_BrakeState_h
#define _ROS_mir_msgs_BrakeState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class BrakeState : public ros::Msg
  {
    public:
      typedef const char* _state_string_type;
      _state_string_type state_string;
      typedef uint8_t _state_type;
      _state_type state;
      typedef bool _braked_type;
      _braked_type braked;

    BrakeState():
      state_string(""),
      state(0),
      braked(0)
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
      } u_braked;
      u_braked.real = this->braked;
      *(outbuffer + offset + 0) = (u_braked.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->braked);
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
      } u_braked;
      u_braked.base = 0;
      u_braked.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->braked = u_braked.real;
      offset += sizeof(this->braked);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/BrakeState"; };
    virtual const char * getMD5() override { return "ef848bae4fc67617e079b91594ce733b"; };

  };

}
#endif
