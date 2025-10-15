#ifndef _ROS_mir_msgs_RobotState_h
#define _ROS_mir_msgs_RobotState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class RobotState : public ros::Msg
  {
    public:
      typedef uint8_t _robotState_type;
      _robotState_type robotState;
      typedef const char* _robotStateString_type;
      _robotStateString_type robotStateString;
      enum { ROBOT_STATE_NONE =  0 };
      enum { ROBOT_STATE_STARTING =  1 };
      enum { ROBOT_STATE_SHUTTINGDOWN =  2 };
      enum { ROBOT_STATE_READY =  3		 };
      enum { ROBOT_STATE_PAUSE =  4		 };
      enum { ROBOT_STATE_EXECUTING =  5		 };
      enum { ROBOT_STATE_ABORTED =  6 };
      enum { ROBOT_STATE_COMPLETED =  7		 };
      enum { ROBOT_STATE_DOCKED =  8		 };
      enum { ROBOT_STATE_DOCKING =  9 };
      enum { ROBOT_STATE_EMERGENCYSTOP =  10	 };
      enum { ROBOT_STATE_MANUALCONTROL =  11	 };
      enum { ROBOT_STATE_ERROR =  12		 };

    RobotState():
      robotState(0),
      robotStateString("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->robotState >> (8 * 0)) & 0xFF;
      offset += sizeof(this->robotState);
      uint32_t length_robotStateString = strlen(this->robotStateString);
      varToArr(outbuffer + offset, length_robotStateString);
      offset += 4;
      memcpy(outbuffer + offset, this->robotStateString, length_robotStateString);
      offset += length_robotStateString;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->robotState =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->robotState);
      uint32_t length_robotStateString;
      arrToVar(length_robotStateString, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_robotStateString; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_robotStateString-1]=0;
      this->robotStateString = (char *)(inbuffer + offset-1);
      offset += length_robotStateString;
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/RobotState"; };
    virtual const char * getMD5() override { return "e9944ef1184bc5d5298157a9fe91aa4e"; };

  };

}
#endif
