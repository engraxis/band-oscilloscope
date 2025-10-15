#ifndef _ROS_mir_msgs_RobotMode_h
#define _ROS_mir_msgs_RobotMode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class RobotMode : public ros::Msg
  {
    public:
      typedef uint8_t _robotMode_type;
      _robotMode_type robotMode;
      typedef const char* _robotModeString_type;
      _robotModeString_type robotModeString;
      enum { ROBOT_MODE_NONE =  0		 };
      enum { ROBOT_MODE_MAPPING =  3		 };
      enum { ROBOT_MODE_MISSION =  7		 };
      enum { ROBOT_MODE_CHANGING =  255		 };

    RobotMode():
      robotMode(0),
      robotModeString("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->robotMode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->robotMode);
      uint32_t length_robotModeString = strlen(this->robotModeString);
      varToArr(outbuffer + offset, length_robotModeString);
      offset += 4;
      memcpy(outbuffer + offset, this->robotModeString, length_robotModeString);
      offset += length_robotModeString;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->robotMode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->robotMode);
      uint32_t length_robotModeString;
      arrToVar(length_robotModeString, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_robotModeString; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_robotModeString-1]=0;
      this->robotModeString = (char *)(inbuffer + offset-1);
      offset += length_robotModeString;
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/RobotMode"; };
    virtual const char * getMD5() override { return "eba8bb1579179193cb02f80018cc79eb"; };

  };

}
#endif
