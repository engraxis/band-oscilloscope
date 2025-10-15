#ifndef _ROS_mir_msgs_Gpio_h
#define _ROS_mir_msgs_Gpio_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class Gpio : public ros::Msg
  {
    public:
      typedef uint8_t _ioport_type;
      _ioport_type ioport;
      typedef uint8_t _dat_type;
      _dat_type dat;
      enum { POWERBOARD_GPIO =  0 };
      enum { POWERBOARD_RESET_SWITCH_LED =  1 };
      enum { PENDANT_INPUT =  5 };
      enum { AUTO_MODE_SWITCH =  10 };
      enum { MANUAL_MODE_SWITCH =  11 };

    Gpio():
      ioport(0),
      dat(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ioport >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ioport);
      *(outbuffer + offset + 0) = (this->dat >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dat);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->ioport =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ioport);
      this->dat =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dat);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/Gpio"; };
    virtual const char * getMD5() override { return "9ea786c6c62a8d8cc7b65489f086f3d3"; };

  };

}
#endif
