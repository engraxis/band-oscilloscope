#ifndef _ROS_SERVICE_GetPosError_h
#define _ROS_SERVICE_GetPosError_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace got_node
{

static const char GETPOSERROR[] = "got_node/GetPosError";

  class GetPosErrorRequest : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;

    GetPosErrorRequest():
      x(0),
      y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
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
        float real;
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
        float real;
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

    virtual const char * getType() override { return GETPOSERROR; };
    virtual const char * getMD5() override { return "ff8d7d66dd3e4b731ef14a45d38888b6"; };

  };

  class GetPosErrorResponse : public ros::Msg
  {
    public:
      typedef float _err_x_type;
      _err_x_type err_x;
      typedef float _err_y_type;
      _err_y_type err_y;

    GetPosErrorResponse():
      err_x(0),
      err_y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_err_x;
      u_err_x.real = this->err_x;
      *(outbuffer + offset + 0) = (u_err_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->err_x);
      union {
        float real;
        uint32_t base;
      } u_err_y;
      u_err_y.real = this->err_y;
      *(outbuffer + offset + 0) = (u_err_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->err_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_err_x;
      u_err_x.base = 0;
      u_err_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->err_x = u_err_x.real;
      offset += sizeof(this->err_x);
      union {
        float real;
        uint32_t base;
      } u_err_y;
      u_err_y.base = 0;
      u_err_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->err_y = u_err_y.real;
      offset += sizeof(this->err_y);
     return offset;
    }

    virtual const char * getType() override { return GETPOSERROR; };
    virtual const char * getMD5() override { return "de03141da676d0a739f43638aa306fa7"; };

  };

  class GetPosError {
    public:
    typedef GetPosErrorRequest Request;
    typedef GetPosErrorResponse Response;
  };

}
#endif
