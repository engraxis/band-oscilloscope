#ifndef _ROS_mir_msgs_MissionCtrlState_h
#define _ROS_mir_msgs_MissionCtrlState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class MissionCtrlState : public ros::Msg
  {
    public:
      typedef int32_t _state_type;
      _state_type state;
      typedef int32_t _pos_id_type;
      _pos_id_type pos_id;
      enum { STATE_IDLE =  0 };
      enum { STATE_WAIT_POS_LOCK =  1 };
      enum { STATE_WAIT_AREA_LOCK =  2 };
      enum { STATE_WAIT_MAP_TRANSITION =  10 };
      enum { STATE_WAIT_LIFT_START_FLOOR =  11 };
      enum { STATE_WAIT_LIFT_END_FLOOR =  12 };

    MissionCtrlState():
      state(0),
      pos_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_state.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state);
      union {
        int32_t real;
        uint32_t base;
      } u_pos_id;
      u_pos_id.real = this->pos_id;
      *(outbuffer + offset + 0) = (u_pos_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pos_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pos_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pos_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->state = u_state.real;
      offset += sizeof(this->state);
      union {
        int32_t real;
        uint32_t base;
      } u_pos_id;
      u_pos_id.base = 0;
      u_pos_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pos_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pos_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pos_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pos_id = u_pos_id.real;
      offset += sizeof(this->pos_id);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/MissionCtrlState"; };
    virtual const char * getMD5() override { return "ebe4dc80faabdec3ee007c4111ff87dd"; };

  };

}
#endif
