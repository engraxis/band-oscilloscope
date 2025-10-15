#ifndef _ROS_mir_msgs_WebPath_h
#define _ROS_mir_msgs_WebPath_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class WebPath : public ros::Msg
  {
    public:
      typedef int32_t _seq_type;
      _seq_type seq;
      uint32_t x_length;
      typedef float _x_type;
      _x_type st_x;
      _x_type * x;
      uint32_t y_length;
      typedef float _y_type;
      _y_type st_y;
      _y_type * y;

    WebPath():
      seq(0),
      x_length(0), st_x(), x(nullptr),
      y_length(0), st_y(), y(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_seq;
      u_seq.real = this->seq;
      *(outbuffer + offset + 0) = (u_seq.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_seq.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_seq.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_seq.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->seq);
      *(outbuffer + offset + 0) = (this->x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_length);
      for( uint32_t i = 0; i < x_length; i++){
      union {
        float real;
        uint32_t base;
      } u_xi;
      u_xi.real = this->x[i];
      *(outbuffer + offset + 0) = (u_xi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x[i]);
      }
      *(outbuffer + offset + 0) = (this->y_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->y_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->y_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->y_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_length);
      for( uint32_t i = 0; i < y_length; i++){
      union {
        float real;
        uint32_t base;
      } u_yi;
      u_yi.real = this->y[i];
      *(outbuffer + offset + 0) = (u_yi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_seq;
      u_seq.base = 0;
      u_seq.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_seq.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_seq.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_seq.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->seq = u_seq.real;
      offset += sizeof(this->seq);
      uint32_t x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->x_length);
      if(x_lengthT > x_length)
        this->x = (float*)realloc(this->x, x_lengthT * sizeof(float));
      x_length = x_lengthT;
      for( uint32_t i = 0; i < x_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_x;
      u_st_x.base = 0;
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_x = u_st_x.real;
      offset += sizeof(this->st_x);
        memcpy( &(this->x[i]), &(this->st_x), sizeof(float));
      }
      uint32_t y_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->y_length);
      if(y_lengthT > y_length)
        this->y = (float*)realloc(this->y, y_lengthT * sizeof(float));
      y_length = y_lengthT;
      for( uint32_t i = 0; i < y_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_y;
      u_st_y.base = 0;
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_y = u_st_y.real;
      offset += sizeof(this->st_y);
        memcpy( &(this->y[i]), &(this->st_y), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/WebPath"; };
    virtual const char * getMD5() override { return "ae30a707d92aabd828375025011b8f41"; };

  };

}
#endif
