#ifndef _ROS_mir_msgs_Trolley_h
#define _ROS_mir_msgs_Trolley_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class Trolley : public ros::Msg
  {
    public:
      typedef int32_t _id_type;
      _id_type id;
      typedef float _length_type;
      _length_type length;
      typedef float _width_type;
      _width_type width;
      typedef float _height_type;
      _height_type height;
      typedef float _offset_locked_wheels_type;
      _offset_locked_wheels_type offset_locked_wheels;

    Trolley():
      id(0),
      length(0),
      width(0),
      height(0),
      offset_locked_wheels(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      union {
        float real;
        uint32_t base;
      } u_length;
      u_length.real = this->length;
      *(outbuffer + offset + 0) = (u_length.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_length.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_length.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_length.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->length);
      union {
        float real;
        uint32_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      union {
        float real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      union {
        float real;
        uint32_t base;
      } u_offset_locked_wheels;
      u_offset_locked_wheels.real = this->offset_locked_wheels;
      *(outbuffer + offset + 0) = (u_offset_locked_wheels.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_offset_locked_wheels.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_offset_locked_wheels.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_offset_locked_wheels.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->offset_locked_wheels);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        float real;
        uint32_t base;
      } u_length;
      u_length.base = 0;
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_length.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->length = u_length.real;
      offset += sizeof(this->length);
      union {
        float real;
        uint32_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
      union {
        float real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      union {
        float real;
        uint32_t base;
      } u_offset_locked_wheels;
      u_offset_locked_wheels.base = 0;
      u_offset_locked_wheels.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_offset_locked_wheels.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_offset_locked_wheels.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_offset_locked_wheels.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->offset_locked_wheels = u_offset_locked_wheels.real;
      offset += sizeof(this->offset_locked_wheels);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/Trolley"; };
    virtual const char * getMD5() override { return "f7f198bfa8fab1128035d129c5beedb7"; };

  };

}
#endif
