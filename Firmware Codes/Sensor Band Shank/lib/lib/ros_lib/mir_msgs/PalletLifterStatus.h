#ifndef _ROS_mir_msgs_PalletLifterStatus_h
#define _ROS_mir_msgs_PalletLifterStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class PalletLifterStatus : public ros::Msg
  {
    public:
      typedef bool _is_enabled_type;
      _is_enabled_type is_enabled;
      typedef uint8_t _state_type;
      _state_type state;
      enum { PALLET_LIFT_STATE_DISABLED =  0 };
      enum { PALLET_LIFT_STATE_MOVING =  1 };
      enum { PALLET_LIFT_STATE_DOWN =  2 };
      enum { PALLET_LIFT_STATE_UP =  3 };

    PalletLifterStatus():
      is_enabled(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_enabled;
      u_is_enabled.real = this->is_enabled;
      *(outbuffer + offset + 0) = (u_is_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_enabled);
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_enabled;
      u_is_enabled.base = 0;
      u_is_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_enabled = u_is_enabled.real;
      offset += sizeof(this->is_enabled);
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/PalletLifterStatus"; };
    virtual const char * getMD5() override { return "1b1c1243f8d5de94c78546d13226600e"; };

  };

}
#endif
