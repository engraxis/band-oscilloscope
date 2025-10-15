#ifndef _ROS_mir_msgs_StampedEncoders_h
#define _ROS_mir_msgs_StampedEncoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mir_msgs/Encoders.h"

namespace mir_msgs
{

  class StampedEncoders : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef mir_msgs::Encoders _encoders_type;
      _encoders_type encoders;

    StampedEncoders():
      header(),
      encoders()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->encoders.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->encoders.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/StampedEncoders"; };
    virtual const char * getMD5() override { return "7c217717e3bf9ebebdee0e043bc42e56"; };

  };

}
#endif
