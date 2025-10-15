#ifndef _ROS_mir_msgs_Devices_h
#define _ROS_mir_msgs_Devices_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mir_msgs/Device.h"

namespace mir_msgs
{

  class Devices : public ros::Msg
  {
    public:
      uint32_t devices_length;
      typedef mir_msgs::Device _devices_type;
      _devices_type st_devices;
      _devices_type * devices;

    Devices():
      devices_length(0), st_devices(), devices(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->devices_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->devices_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->devices_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->devices_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->devices_length);
      for( uint32_t i = 0; i < devices_length; i++){
      offset += this->devices[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t devices_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->devices_length);
      if(devices_lengthT > devices_length)
        this->devices = (mir_msgs::Device*)realloc(this->devices, devices_lengthT * sizeof(mir_msgs::Device));
      devices_length = devices_lengthT;
      for( uint32_t i = 0; i < devices_length; i++){
      offset += this->st_devices.deserialize(inbuffer + offset);
        memcpy( &(this->devices[i]), &(this->st_devices), sizeof(mir_msgs::Device));
      }
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/Devices"; };
    virtual const char * getMD5() override { return "511b1be8e995256c8e1402bcafc15e2b"; };

  };

}
#endif
