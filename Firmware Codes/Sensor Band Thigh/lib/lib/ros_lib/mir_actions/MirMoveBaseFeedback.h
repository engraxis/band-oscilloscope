#ifndef _ROS_mir_actions_MirMoveBaseFeedback_h
#define _ROS_mir_actions_MirMoveBaseFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace mir_actions
{

  class MirMoveBaseFeedback : public ros::Msg
  {
    public:
      typedef int8_t _state_type;
      _state_type state;
      typedef geometry_msgs::PoseStamped _base_position_type;
      _base_position_type base_position;
      enum { PLANNING =  0 };
      enum { CONTROLLING =  1 };
      enum { CLEARING =  2 };

    MirMoveBaseFeedback():
      state(0),
      base_position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      offset += this->base_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
      offset += this->base_position.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_actions/MirMoveBaseFeedback"; };
    virtual const char * getMD5() override { return "444e2bb5720367d489f77ad420c36753"; };

  };

}
#endif
