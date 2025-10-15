#ifndef _ROS_SERVICE_Flags_h
#define _ROS_SERVICE_Flags_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sdc21x0
{

static const char FLAGS[] = "sdc21x0/Flags";

  class FlagsRequest : public ros::Msg
  {
    public:
      typedef int32_t _digitalPort_type;
      _digitalPort_type digitalPort;

    FlagsRequest():
      digitalPort(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_digitalPort;
      u_digitalPort.real = this->digitalPort;
      *(outbuffer + offset + 0) = (u_digitalPort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_digitalPort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_digitalPort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_digitalPort.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->digitalPort);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_digitalPort;
      u_digitalPort.base = 0;
      u_digitalPort.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_digitalPort.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_digitalPort.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_digitalPort.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->digitalPort = u_digitalPort.real;
      offset += sizeof(this->digitalPort);
     return offset;
    }

    virtual const char * getType() override { return FLAGS; };
    virtual const char * getMD5() override { return "a48556dca33c04c7d060ef1e26808db0"; };

  };

  class FlagsResponse : public ros::Msg
  {
    public:
      typedef bool _response_type;
      _response_type response;

    FlagsResponse():
      response(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_response;
      u_response.real = this->response;
      *(outbuffer + offset + 0) = (u_response.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->response);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_response;
      u_response.base = 0;
      u_response.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->response = u_response.real;
      offset += sizeof(this->response);
     return offset;
    }

    virtual const char * getType() override { return FLAGS; };
    virtual const char * getMD5() override { return "003b81baa95ab323fc1ddf3c7d0bee81"; };

  };

  class Flags {
    public:
    typedef FlagsRequest Request;
    typedef FlagsResponse Response;
  };

}
#endif
