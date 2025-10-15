#ifndef _ROS_mir_msgs_Device_h
#define _ROS_mir_msgs_Device_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class Device : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _serial_type;
      _serial_type serial;

    Device():
      name(""),
      serial("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_serial = strlen(this->serial);
      varToArr(outbuffer + offset, length_serial);
      offset += 4;
      memcpy(outbuffer + offset, this->serial, length_serial);
      offset += length_serial;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_serial;
      arrToVar(length_serial, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_serial; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_serial-1]=0;
      this->serial = (char *)(inbuffer + offset-1);
      offset += length_serial;
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/Device"; };
    virtual const char * getMD5() override { return "4914eb207f0463464c48e14410d8a949"; };

  };

}
#endif
