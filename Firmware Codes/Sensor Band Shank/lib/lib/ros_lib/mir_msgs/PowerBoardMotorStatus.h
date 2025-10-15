#ifndef _ROS_mir_msgs_PowerBoardMotorStatus_h
#define _ROS_mir_msgs_PowerBoardMotorStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class PowerBoardMotorStatus : public ros::Msg
  {
    public:
      typedef uint16_t _LeftMotor_CtrlWord_type;
      _LeftMotor_CtrlWord_type LeftMotor_CtrlWord;
      typedef int32_t _LeftMotor_Speed_type;
      _LeftMotor_Speed_type LeftMotor_Speed;
      typedef int32_t _LeftMotor_Encoder_type;
      _LeftMotor_Encoder_type LeftMotor_Encoder;
      typedef uint16_t _LeftMotor_Status_type;
      _LeftMotor_Status_type LeftMotor_Status;
      typedef uint8_t _LeftMotor_Error_type;
      _LeftMotor_Error_type LeftMotor_Error;
      typedef uint32_t _LeftMotor_ErrorHist1_type;
      _LeftMotor_ErrorHist1_type LeftMotor_ErrorHist1;
      typedef uint32_t _LeftMotor_ErrorHist2_type;
      _LeftMotor_ErrorHist2_type LeftMotor_ErrorHist2;
      typedef int32_t _LeftMotor_Current_type;
      _LeftMotor_Current_type LeftMotor_Current;
      typedef uint16_t _LeftMotor_I2t_Motor_type;
      _LeftMotor_I2t_Motor_type LeftMotor_I2t_Motor;
      typedef uint16_t _LeftMotor_I2t_Controller_type;
      _LeftMotor_I2t_Controller_type LeftMotor_I2t_Controller;
      typedef int16_t _LeftMotor_Temperature_type;
      _LeftMotor_Temperature_type LeftMotor_Temperature;
      typedef uint16_t _RightMotor_CtrlWord_type;
      _RightMotor_CtrlWord_type RightMotor_CtrlWord;
      typedef int32_t _RightMotor_Speed_type;
      _RightMotor_Speed_type RightMotor_Speed;
      typedef int32_t _RightMotor_Encoder_type;
      _RightMotor_Encoder_type RightMotor_Encoder;
      typedef uint16_t _RightMotor_Status_type;
      _RightMotor_Status_type RightMotor_Status;
      typedef uint8_t _RightMotor_Error_type;
      _RightMotor_Error_type RightMotor_Error;
      typedef uint32_t _RightMotor_ErrorHist1_type;
      _RightMotor_ErrorHist1_type RightMotor_ErrorHist1;
      typedef uint32_t _RightMotor_ErrorHist2_type;
      _RightMotor_ErrorHist2_type RightMotor_ErrorHist2;
      typedef int32_t _RightMotor_Current_type;
      _RightMotor_Current_type RightMotor_Current;
      typedef uint16_t _RightMotor_I2t_Motor_type;
      _RightMotor_I2t_Motor_type RightMotor_I2t_Motor;
      typedef uint16_t _RightMotor_I2t_Controller_type;
      _RightMotor_I2t_Controller_type RightMotor_I2t_Controller;
      typedef int16_t _RightMotor_Temperature_type;
      _RightMotor_Temperature_type RightMotor_Temperature;
      typedef uint8_t _Brake_LeftStatus_type;
      _Brake_LeftStatus_type Brake_LeftStatus;
      typedef uint8_t _Brake_RightStatus_type;
      _Brake_RightStatus_type Brake_RightStatus;

    PowerBoardMotorStatus():
      LeftMotor_CtrlWord(0),
      LeftMotor_Speed(0),
      LeftMotor_Encoder(0),
      LeftMotor_Status(0),
      LeftMotor_Error(0),
      LeftMotor_ErrorHist1(0),
      LeftMotor_ErrorHist2(0),
      LeftMotor_Current(0),
      LeftMotor_I2t_Motor(0),
      LeftMotor_I2t_Controller(0),
      LeftMotor_Temperature(0),
      RightMotor_CtrlWord(0),
      RightMotor_Speed(0),
      RightMotor_Encoder(0),
      RightMotor_Status(0),
      RightMotor_Error(0),
      RightMotor_ErrorHist1(0),
      RightMotor_ErrorHist2(0),
      RightMotor_Current(0),
      RightMotor_I2t_Motor(0),
      RightMotor_I2t_Controller(0),
      RightMotor_Temperature(0),
      Brake_LeftStatus(0),
      Brake_RightStatus(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->LeftMotor_CtrlWord >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->LeftMotor_CtrlWord >> (8 * 1)) & 0xFF;
      offset += sizeof(this->LeftMotor_CtrlWord);
      union {
        int32_t real;
        uint32_t base;
      } u_LeftMotor_Speed;
      u_LeftMotor_Speed.real = this->LeftMotor_Speed;
      *(outbuffer + offset + 0) = (u_LeftMotor_Speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_LeftMotor_Speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_LeftMotor_Speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_LeftMotor_Speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->LeftMotor_Speed);
      union {
        int32_t real;
        uint32_t base;
      } u_LeftMotor_Encoder;
      u_LeftMotor_Encoder.real = this->LeftMotor_Encoder;
      *(outbuffer + offset + 0) = (u_LeftMotor_Encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_LeftMotor_Encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_LeftMotor_Encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_LeftMotor_Encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->LeftMotor_Encoder);
      *(outbuffer + offset + 0) = (this->LeftMotor_Status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->LeftMotor_Status >> (8 * 1)) & 0xFF;
      offset += sizeof(this->LeftMotor_Status);
      *(outbuffer + offset + 0) = (this->LeftMotor_Error >> (8 * 0)) & 0xFF;
      offset += sizeof(this->LeftMotor_Error);
      *(outbuffer + offset + 0) = (this->LeftMotor_ErrorHist1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->LeftMotor_ErrorHist1 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->LeftMotor_ErrorHist1 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->LeftMotor_ErrorHist1 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->LeftMotor_ErrorHist1);
      *(outbuffer + offset + 0) = (this->LeftMotor_ErrorHist2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->LeftMotor_ErrorHist2 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->LeftMotor_ErrorHist2 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->LeftMotor_ErrorHist2 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->LeftMotor_ErrorHist2);
      union {
        int32_t real;
        uint32_t base;
      } u_LeftMotor_Current;
      u_LeftMotor_Current.real = this->LeftMotor_Current;
      *(outbuffer + offset + 0) = (u_LeftMotor_Current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_LeftMotor_Current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_LeftMotor_Current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_LeftMotor_Current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->LeftMotor_Current);
      *(outbuffer + offset + 0) = (this->LeftMotor_I2t_Motor >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->LeftMotor_I2t_Motor >> (8 * 1)) & 0xFF;
      offset += sizeof(this->LeftMotor_I2t_Motor);
      *(outbuffer + offset + 0) = (this->LeftMotor_I2t_Controller >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->LeftMotor_I2t_Controller >> (8 * 1)) & 0xFF;
      offset += sizeof(this->LeftMotor_I2t_Controller);
      union {
        int16_t real;
        uint16_t base;
      } u_LeftMotor_Temperature;
      u_LeftMotor_Temperature.real = this->LeftMotor_Temperature;
      *(outbuffer + offset + 0) = (u_LeftMotor_Temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_LeftMotor_Temperature.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->LeftMotor_Temperature);
      *(outbuffer + offset + 0) = (this->RightMotor_CtrlWord >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RightMotor_CtrlWord >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RightMotor_CtrlWord);
      union {
        int32_t real;
        uint32_t base;
      } u_RightMotor_Speed;
      u_RightMotor_Speed.real = this->RightMotor_Speed;
      *(outbuffer + offset + 0) = (u_RightMotor_Speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_RightMotor_Speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_RightMotor_Speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_RightMotor_Speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->RightMotor_Speed);
      union {
        int32_t real;
        uint32_t base;
      } u_RightMotor_Encoder;
      u_RightMotor_Encoder.real = this->RightMotor_Encoder;
      *(outbuffer + offset + 0) = (u_RightMotor_Encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_RightMotor_Encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_RightMotor_Encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_RightMotor_Encoder.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->RightMotor_Encoder);
      *(outbuffer + offset + 0) = (this->RightMotor_Status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RightMotor_Status >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RightMotor_Status);
      *(outbuffer + offset + 0) = (this->RightMotor_Error >> (8 * 0)) & 0xFF;
      offset += sizeof(this->RightMotor_Error);
      *(outbuffer + offset + 0) = (this->RightMotor_ErrorHist1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RightMotor_ErrorHist1 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->RightMotor_ErrorHist1 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->RightMotor_ErrorHist1 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->RightMotor_ErrorHist1);
      *(outbuffer + offset + 0) = (this->RightMotor_ErrorHist2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RightMotor_ErrorHist2 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->RightMotor_ErrorHist2 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->RightMotor_ErrorHist2 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->RightMotor_ErrorHist2);
      union {
        int32_t real;
        uint32_t base;
      } u_RightMotor_Current;
      u_RightMotor_Current.real = this->RightMotor_Current;
      *(outbuffer + offset + 0) = (u_RightMotor_Current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_RightMotor_Current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_RightMotor_Current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_RightMotor_Current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->RightMotor_Current);
      *(outbuffer + offset + 0) = (this->RightMotor_I2t_Motor >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RightMotor_I2t_Motor >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RightMotor_I2t_Motor);
      *(outbuffer + offset + 0) = (this->RightMotor_I2t_Controller >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->RightMotor_I2t_Controller >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RightMotor_I2t_Controller);
      union {
        int16_t real;
        uint16_t base;
      } u_RightMotor_Temperature;
      u_RightMotor_Temperature.real = this->RightMotor_Temperature;
      *(outbuffer + offset + 0) = (u_RightMotor_Temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_RightMotor_Temperature.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->RightMotor_Temperature);
      *(outbuffer + offset + 0) = (this->Brake_LeftStatus >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Brake_LeftStatus);
      *(outbuffer + offset + 0) = (this->Brake_RightStatus >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Brake_RightStatus);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->LeftMotor_CtrlWord =  ((uint16_t) (*(inbuffer + offset)));
      this->LeftMotor_CtrlWord |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->LeftMotor_CtrlWord);
      union {
        int32_t real;
        uint32_t base;
      } u_LeftMotor_Speed;
      u_LeftMotor_Speed.base = 0;
      u_LeftMotor_Speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_LeftMotor_Speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_LeftMotor_Speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_LeftMotor_Speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->LeftMotor_Speed = u_LeftMotor_Speed.real;
      offset += sizeof(this->LeftMotor_Speed);
      union {
        int32_t real;
        uint32_t base;
      } u_LeftMotor_Encoder;
      u_LeftMotor_Encoder.base = 0;
      u_LeftMotor_Encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_LeftMotor_Encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_LeftMotor_Encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_LeftMotor_Encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->LeftMotor_Encoder = u_LeftMotor_Encoder.real;
      offset += sizeof(this->LeftMotor_Encoder);
      this->LeftMotor_Status =  ((uint16_t) (*(inbuffer + offset)));
      this->LeftMotor_Status |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->LeftMotor_Status);
      this->LeftMotor_Error =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->LeftMotor_Error);
      this->LeftMotor_ErrorHist1 =  ((uint32_t) (*(inbuffer + offset)));
      this->LeftMotor_ErrorHist1 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->LeftMotor_ErrorHist1 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->LeftMotor_ErrorHist1 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->LeftMotor_ErrorHist1);
      this->LeftMotor_ErrorHist2 =  ((uint32_t) (*(inbuffer + offset)));
      this->LeftMotor_ErrorHist2 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->LeftMotor_ErrorHist2 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->LeftMotor_ErrorHist2 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->LeftMotor_ErrorHist2);
      union {
        int32_t real;
        uint32_t base;
      } u_LeftMotor_Current;
      u_LeftMotor_Current.base = 0;
      u_LeftMotor_Current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_LeftMotor_Current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_LeftMotor_Current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_LeftMotor_Current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->LeftMotor_Current = u_LeftMotor_Current.real;
      offset += sizeof(this->LeftMotor_Current);
      this->LeftMotor_I2t_Motor =  ((uint16_t) (*(inbuffer + offset)));
      this->LeftMotor_I2t_Motor |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->LeftMotor_I2t_Motor);
      this->LeftMotor_I2t_Controller =  ((uint16_t) (*(inbuffer + offset)));
      this->LeftMotor_I2t_Controller |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->LeftMotor_I2t_Controller);
      union {
        int16_t real;
        uint16_t base;
      } u_LeftMotor_Temperature;
      u_LeftMotor_Temperature.base = 0;
      u_LeftMotor_Temperature.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_LeftMotor_Temperature.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->LeftMotor_Temperature = u_LeftMotor_Temperature.real;
      offset += sizeof(this->LeftMotor_Temperature);
      this->RightMotor_CtrlWord =  ((uint16_t) (*(inbuffer + offset)));
      this->RightMotor_CtrlWord |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->RightMotor_CtrlWord);
      union {
        int32_t real;
        uint32_t base;
      } u_RightMotor_Speed;
      u_RightMotor_Speed.base = 0;
      u_RightMotor_Speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_RightMotor_Speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_RightMotor_Speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_RightMotor_Speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->RightMotor_Speed = u_RightMotor_Speed.real;
      offset += sizeof(this->RightMotor_Speed);
      union {
        int32_t real;
        uint32_t base;
      } u_RightMotor_Encoder;
      u_RightMotor_Encoder.base = 0;
      u_RightMotor_Encoder.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_RightMotor_Encoder.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_RightMotor_Encoder.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_RightMotor_Encoder.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->RightMotor_Encoder = u_RightMotor_Encoder.real;
      offset += sizeof(this->RightMotor_Encoder);
      this->RightMotor_Status =  ((uint16_t) (*(inbuffer + offset)));
      this->RightMotor_Status |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->RightMotor_Status);
      this->RightMotor_Error =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->RightMotor_Error);
      this->RightMotor_ErrorHist1 =  ((uint32_t) (*(inbuffer + offset)));
      this->RightMotor_ErrorHist1 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->RightMotor_ErrorHist1 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->RightMotor_ErrorHist1 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->RightMotor_ErrorHist1);
      this->RightMotor_ErrorHist2 =  ((uint32_t) (*(inbuffer + offset)));
      this->RightMotor_ErrorHist2 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->RightMotor_ErrorHist2 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->RightMotor_ErrorHist2 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->RightMotor_ErrorHist2);
      union {
        int32_t real;
        uint32_t base;
      } u_RightMotor_Current;
      u_RightMotor_Current.base = 0;
      u_RightMotor_Current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_RightMotor_Current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_RightMotor_Current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_RightMotor_Current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->RightMotor_Current = u_RightMotor_Current.real;
      offset += sizeof(this->RightMotor_Current);
      this->RightMotor_I2t_Motor =  ((uint16_t) (*(inbuffer + offset)));
      this->RightMotor_I2t_Motor |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->RightMotor_I2t_Motor);
      this->RightMotor_I2t_Controller =  ((uint16_t) (*(inbuffer + offset)));
      this->RightMotor_I2t_Controller |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->RightMotor_I2t_Controller);
      union {
        int16_t real;
        uint16_t base;
      } u_RightMotor_Temperature;
      u_RightMotor_Temperature.base = 0;
      u_RightMotor_Temperature.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_RightMotor_Temperature.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->RightMotor_Temperature = u_RightMotor_Temperature.real;
      offset += sizeof(this->RightMotor_Temperature);
      this->Brake_LeftStatus =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Brake_LeftStatus);
      this->Brake_RightStatus =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Brake_RightStatus);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/PowerBoardMotorStatus"; };
    virtual const char * getMD5() override { return "6fa84650aa5369a4240fff080dbf7c68"; };

  };

}
#endif
