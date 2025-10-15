#ifndef _ROS_mir_msgs_HookExtendedStatus_h
#define _ROS_mir_msgs_HookExtendedStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mir_msgs/BrakeState.h"
#include "mir_msgs/GripperState.h"
#include "mir_msgs/HeightState.h"

namespace mir_msgs
{

  class HookExtendedStatus : public ros::Msg
  {
    public:
      typedef bool _available_type;
      _available_type available;
      typedef mir_msgs::BrakeState _brake_type;
      _brake_type brake;
      typedef mir_msgs::GripperState _gripper_type;
      _gripper_type gripper;
      typedef mir_msgs::HeightState _height_type;
      _height_type height;
      typedef float _angle_type;
      _angle_type angle;
      typedef const char* _qr_marker_name_type;
      _qr_marker_name_type qr_marker_name;

    HookExtendedStatus():
      available(0),
      brake(),
      gripper(),
      height(),
      angle(0),
      qr_marker_name("")
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
      offset += this->brake.serialize(outbuffer + offset);
      offset += this->gripper.serialize(outbuffer + offset);
      offset += this->height.serialize(outbuffer + offset);
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
      uint32_t length_qr_marker_name = strlen(this->qr_marker_name);
      varToArr(outbuffer + offset, length_qr_marker_name);
      offset += 4;
      memcpy(outbuffer + offset, this->qr_marker_name, length_qr_marker_name);
      offset += length_qr_marker_name;
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
      offset += this->brake.deserialize(inbuffer + offset);
      offset += this->gripper.deserialize(inbuffer + offset);
      offset += this->height.deserialize(inbuffer + offset);
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
      uint32_t length_qr_marker_name;
      arrToVar(length_qr_marker_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_qr_marker_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_qr_marker_name-1]=0;
      this->qr_marker_name = (char *)(inbuffer + offset-1);
      offset += length_qr_marker_name;
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/HookExtendedStatus"; };
    virtual const char * getMD5() override { return "c879cdfcaceab2b74790f2d69b0a637f"; };

  };

}
#endif
