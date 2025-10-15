#ifndef _ROS_mir_msgs_MirExtra_h
#define _ROS_mir_msgs_MirExtra_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mir_msgs
{

  class MirExtra : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _time_delta_type;
      _time_delta_type time_delta;
      typedef float _r_rpm_type;
      _r_rpm_type r_rpm;
      typedef float _l_rpm_type;
      _l_rpm_type l_rpm;
      typedef float _vel_type;
      _vel_type vel;
      typedef float _ang_type;
      _ang_type ang;

    MirExtra():
      header(),
      time_delta(0),
      r_rpm(0),
      l_rpm(0),
      vel(0),
      ang(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_time_delta;
      u_time_delta.real = this->time_delta;
      *(outbuffer + offset + 0) = (u_time_delta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time_delta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time_delta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time_delta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_delta);
      union {
        float real;
        uint32_t base;
      } u_r_rpm;
      u_r_rpm.real = this->r_rpm;
      *(outbuffer + offset + 0) = (u_r_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_rpm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r_rpm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r_rpm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_rpm);
      union {
        float real;
        uint32_t base;
      } u_l_rpm;
      u_l_rpm.real = this->l_rpm;
      *(outbuffer + offset + 0) = (u_l_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_rpm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_l_rpm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_l_rpm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_rpm);
      union {
        float real;
        uint32_t base;
      } u_vel;
      u_vel.real = this->vel;
      *(outbuffer + offset + 0) = (u_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel);
      union {
        float real;
        uint32_t base;
      } u_ang;
      u_ang.real = this->ang;
      *(outbuffer + offset + 0) = (u_ang.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ang.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ang.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ang.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ang);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_time_delta;
      u_time_delta.base = 0;
      u_time_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->time_delta = u_time_delta.real;
      offset += sizeof(this->time_delta);
      union {
        float real;
        uint32_t base;
      } u_r_rpm;
      u_r_rpm.base = 0;
      u_r_rpm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r_rpm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r_rpm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r_rpm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r_rpm = u_r_rpm.real;
      offset += sizeof(this->r_rpm);
      union {
        float real;
        uint32_t base;
      } u_l_rpm;
      u_l_rpm.base = 0;
      u_l_rpm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_l_rpm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_l_rpm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_l_rpm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->l_rpm = u_l_rpm.real;
      offset += sizeof(this->l_rpm);
      union {
        float real;
        uint32_t base;
      } u_vel;
      u_vel.base = 0;
      u_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel = u_vel.real;
      offset += sizeof(this->vel);
      union {
        float real;
        uint32_t base;
      } u_ang;
      u_ang.base = 0;
      u_ang.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ang.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ang.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ang.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ang = u_ang.real;
      offset += sizeof(this->ang);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/MirExtra"; };
    virtual const char * getMD5() override { return "70adfdf09e98057d681bf9b0e6251bbf"; };

  };

}
#endif
