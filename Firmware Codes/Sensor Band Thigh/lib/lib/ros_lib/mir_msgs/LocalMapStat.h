#ifndef _ROS_mir_msgs_LocalMapStat_h
#define _ROS_mir_msgs_LocalMapStat_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class LocalMapStat : public ros::Msg
  {
    public:
      typedef int32_t _idx_type;
      _idx_type idx;
      typedef int32_t _x_type;
      _x_type x;
      typedef int32_t _y_type;
      _y_type y;

    LocalMapStat():
      idx(0),
      x(0),
      y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_idx;
      u_idx.real = this->idx;
      *(outbuffer + offset + 0) = (u_idx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_idx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_idx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_idx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->idx);
      union {
        int32_t real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        int32_t real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_idx;
      u_idx.base = 0;
      u_idx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_idx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_idx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_idx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->idx = u_idx.real;
      offset += sizeof(this->idx);
      union {
        int32_t real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        int32_t real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/LocalMapStat"; };
    virtual const char * getMD5() override { return "b4421362b1eb70c688ba12172d1d8f04"; };

  };

}
#endif
