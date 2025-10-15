#ifndef _ROS_mir_msgs_HookStatus_h
#define _ROS_mir_msgs_HookStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mir_msgs/Trolley.h"

namespace mir_msgs
{

  class HookStatus : public ros::Msg
  {
    public:
      typedef bool _available_type;
      _available_type available;
      typedef float _length_type;
      _length_type length;
      typedef float _height_type;
      _height_type height;
      typedef float _angle_type;
      _angle_type angle;
      typedef bool _braked_type;
      _braked_type braked;
      typedef bool _trolley_attached_type;
      _trolley_attached_type trolley_attached;
      typedef mir_msgs::Trolley _trolley_type;
      _trolley_type trolley;

    HookStatus():
      available(0),
      length(0),
      height(0),
      angle(0),
      braked(0),
      trolley_attached(0),
      trolley()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_available;
      u_available.real = this->available;
      *(outbuffer + offset + 0) = (u_available.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->available);
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
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle);
      union {
        bool real;
        uint8_t base;
      } u_braked;
      u_braked.real = this->braked;
      *(outbuffer + offset + 0) = (u_braked.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->braked);
      union {
        bool real;
        uint8_t base;
      } u_trolley_attached;
      u_trolley_attached.real = this->trolley_attached;
      *(outbuffer + offset + 0) = (u_trolley_attached.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->trolley_attached);
      offset += this->trolley.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_available;
      u_available.base = 0;
      u_available.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->available = u_available.real;
      offset += sizeof(this->available);
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
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      union {
        bool real;
        uint8_t base;
      } u_braked;
      u_braked.base = 0;
      u_braked.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->braked = u_braked.real;
      offset += sizeof(this->braked);
      union {
        bool real;
        uint8_t base;
      } u_trolley_attached;
      u_trolley_attached.base = 0;
      u_trolley_attached.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->trolley_attached = u_trolley_attached.real;
      offset += sizeof(this->trolley_attached);
      offset += this->trolley.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/HookStatus"; };
    virtual const char * getMD5() override { return "1bce86e4d0caff20e36c78d3bd47f560"; };

  };

}
#endif
