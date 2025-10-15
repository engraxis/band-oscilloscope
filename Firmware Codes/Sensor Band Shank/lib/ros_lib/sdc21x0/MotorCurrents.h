#ifndef _ROS_sdc21x0_MotorCurrents_h
#define _ROS_sdc21x0_MotorCurrents_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sdc21x0
{

  class MotorCurrents : public ros::Msg
  {
    public:
      typedef float _left_motor_type;
      _left_motor_type left_motor;
      typedef float _right_motor_type;
      _right_motor_type right_motor;

    MotorCurrents():
      left_motor(0),
      right_motor(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_motor;
      u_left_motor.real = this->left_motor;
      *(outbuffer + offset + 0) = (u_left_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_motor);
      union {
        float real;
        uint32_t base;
      } u_right_motor;
      u_right_motor.real = this->right_motor;
      *(outbuffer + offset + 0) = (u_right_motor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_motor);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_motor;
      u_left_motor.base = 0;
      u_left_motor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_motor = u_left_motor.real;
      offset += sizeof(this->left_motor);
      union {
        float real;
        uint32_t base;
      } u_right_motor;
      u_right_motor.base = 0;
      u_right_motor.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_motor = u_right_motor.real;
      offset += sizeof(this->right_motor);
     return offset;
    }

    virtual const char * getType() override { return "sdc21x0/MotorCurrents"; };
    virtual const char * getMD5() override { return "3e3717ac8e9443aa62d7102a5860f5e7"; };

  };

}
#endif
