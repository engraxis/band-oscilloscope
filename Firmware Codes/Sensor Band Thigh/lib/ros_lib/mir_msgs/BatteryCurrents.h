#ifndef _ROS_mir_msgs_BatteryCurrents_h
#define _ROS_mir_msgs_BatteryCurrents_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class BatteryCurrents : public ros::Msg
  {
    public:
      typedef float _battery1_current_type;
      _battery1_current_type battery1_current;
      typedef float _battery2_current_type;
      _battery2_current_type battery2_current;

    BatteryCurrents():
      battery1_current(0),
      battery2_current(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->battery1_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->battery2_current);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->battery1_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->battery2_current));
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/BatteryCurrents"; };
    virtual const char * getMD5() override { return "99e76fe5e1c8183e9d7ded8c13ebdf16"; };

  };

}
#endif
