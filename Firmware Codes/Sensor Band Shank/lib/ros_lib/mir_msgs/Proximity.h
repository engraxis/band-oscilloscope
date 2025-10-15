#ifndef _ROS_mir_msgs_Proximity_h
#define _ROS_mir_msgs_Proximity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mir_msgs
{

  class Proximity : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t ranges_length;
      typedef uint16_t _ranges_type;
      _ranges_type st_ranges;
      _ranges_type * ranges;

    Proximity():
      header(),
      ranges_length(0), st_ranges(), ranges(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->ranges_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ranges_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ranges_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ranges_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ranges_length);
      for( uint32_t i = 0; i < ranges_length; i++){
      *(outbuffer + offset + 0) = (this->ranges[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ranges[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ranges[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t ranges_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ranges_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ranges_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ranges_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ranges_length);
      if(ranges_lengthT > ranges_length)
        this->ranges = (uint16_t*)realloc(this->ranges, ranges_lengthT * sizeof(uint16_t));
      ranges_length = ranges_lengthT;
      for( uint32_t i = 0; i < ranges_length; i++){
      this->st_ranges =  ((uint16_t) (*(inbuffer + offset)));
      this->st_ranges |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_ranges);
        memcpy( &(this->ranges[i]), &(this->st_ranges), sizeof(uint16_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/Proximity"; };
    virtual const char * getMD5() override { return "4a0f829b44abd05395872595e09e67f4"; };

  };

}
#endif
