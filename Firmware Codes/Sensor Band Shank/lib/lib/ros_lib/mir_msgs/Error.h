#ifndef _ROS_mir_msgs_Error_h
#define _ROS_mir_msgs_Error_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace mir_msgs
{

  class Error : public ros::Msg
  {
    public:
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;
      typedef int32_t _code_type;
      _code_type code;
      typedef const char* _description_type;
      _description_type description;
      typedef const char* _module_type;
      _module_type module;
      enum { HARDWARE_ERROR =  0 };
      enum { CPU_LOAD_ERROR =  100 };
      enum { MEMORY_ERROR =  200 };
      enum { ETHERNET_ERROR =  300 };
      enum { HDD_ERROR =  400 };
      enum { BATTERY_ERROR =  500 };
      enum { IMU_ERROR =  600 };
      enum { MOTOR_ERROR =  700 };
      enum { LASER_ERROR =  800 };
      enum { CAMERA_ERROR =  900 };
      enum { SAFETY_SYSTEM_ERROR =  1000 };
      enum { HOOK_ERROR =  5000 };
      enum { HOOK_CAMERA_ERROR =  5100 };
      enum { HOOK_ACTUATOR_ERROR =  5200 };
      enum { HOOK_BRAKE_ERROR =  5300 };
      enum { HOOK_ENCODER_ERROR =  5400 };
      enum { MISSING_ERROR =  9000 };
      enum { SOFTWARE_ERROR =  10000 };
      enum { MISSION_ERROR =  10100 };
      enum { LOCALIZATION_ERROR =  10200 };
      enum { MAPPING_ERROR =  10300 };
      enum { ODOM_FUSION_ERROR =  10400 };

    Error():
      timestamp(),
      code(0),
      description(""),
      module("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
      union {
        int32_t real;
        uint32_t base;
      } u_code;
      u_code.real = this->code;
      *(outbuffer + offset + 0) = (u_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_code.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_code.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_code.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->code);
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      uint32_t length_module = strlen(this->module);
      varToArr(outbuffer + offset, length_module);
      offset += 4;
      memcpy(outbuffer + offset, this->module, length_module);
      offset += length_module;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
      union {
        int32_t real;
        uint32_t base;
      } u_code;
      u_code.base = 0;
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_code.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->code = u_code.real;
      offset += sizeof(this->code);
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      uint32_t length_module;
      arrToVar(length_module, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_module; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_module-1]=0;
      this->module = (char *)(inbuffer + offset-1);
      offset += length_module;
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/Error"; };
    virtual const char * getMD5() override { return "5386c06f8c9eb4ee65da90d23441ab00"; };

  };

}
#endif
