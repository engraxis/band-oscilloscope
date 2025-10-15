#ifndef _ROS_mir_msgs_SafetyStatus_h
#define _ROS_mir_msgs_SafetyStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class SafetyStatus : public ros::Msg
  {
    public:
      typedef bool _is_connected_type;
      _is_connected_type is_connected;
      typedef bool _is_firmware_ok_type;
      _is_firmware_ok_type is_firmware_ok;
      typedef int32_t _firmware_version_type;
      _firmware_version_type firmware_version;
      typedef bool _in_protective_stop_type;
      _in_protective_stop_type in_protective_stop;
      typedef bool _in_emergency_stop_type;
      _in_emergency_stop_type in_emergency_stop;
      typedef bool _sto_feedback_type;
      _sto_feedback_type sto_feedback;
      typedef bool _is_restart_required_type;
      _is_restart_required_type is_restart_required;
      typedef bool _is_safety_muted_type;
      _is_safety_muted_type is_safety_muted;
      typedef bool _is_limited_speed_active_type;
      _is_limited_speed_active_type is_limited_speed_active;
      typedef bool _in_sleep_mode_type;
      _in_sleep_mode_type in_sleep_mode;
      typedef bool _in_manual_mode_type;
      _in_manual_mode_type in_manual_mode;
      typedef bool _is_manual_mode_restart_required_type;
      _is_manual_mode_restart_required_type is_manual_mode_restart_required;

    SafetyStatus():
      is_connected(0),
      is_firmware_ok(0),
      firmware_version(0),
      in_protective_stop(0),
      in_emergency_stop(0),
      sto_feedback(0),
      is_restart_required(0),
      is_safety_muted(0),
      is_limited_speed_active(0),
      in_sleep_mode(0),
      in_manual_mode(0),
      is_manual_mode_restart_required(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_connected;
      u_is_connected.real = this->is_connected;
      *(outbuffer + offset + 0) = (u_is_connected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_connected);
      union {
        bool real;
        uint8_t base;
      } u_is_firmware_ok;
      u_is_firmware_ok.real = this->is_firmware_ok;
      *(outbuffer + offset + 0) = (u_is_firmware_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_firmware_ok);
      union {
        int32_t real;
        uint32_t base;
      } u_firmware_version;
      u_firmware_version.real = this->firmware_version;
      *(outbuffer + offset + 0) = (u_firmware_version.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_firmware_version.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_firmware_version.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_firmware_version.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->firmware_version);
      union {
        bool real;
        uint8_t base;
      } u_in_protective_stop;
      u_in_protective_stop.real = this->in_protective_stop;
      *(outbuffer + offset + 0) = (u_in_protective_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->in_protective_stop);
      union {
        bool real;
        uint8_t base;
      } u_in_emergency_stop;
      u_in_emergency_stop.real = this->in_emergency_stop;
      *(outbuffer + offset + 0) = (u_in_emergency_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->in_emergency_stop);
      union {
        bool real;
        uint8_t base;
      } u_sto_feedback;
      u_sto_feedback.real = this->sto_feedback;
      *(outbuffer + offset + 0) = (u_sto_feedback.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sto_feedback);
      union {
        bool real;
        uint8_t base;
      } u_is_restart_required;
      u_is_restart_required.real = this->is_restart_required;
      *(outbuffer + offset + 0) = (u_is_restart_required.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_restart_required);
      union {
        bool real;
        uint8_t base;
      } u_is_safety_muted;
      u_is_safety_muted.real = this->is_safety_muted;
      *(outbuffer + offset + 0) = (u_is_safety_muted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_safety_muted);
      union {
        bool real;
        uint8_t base;
      } u_is_limited_speed_active;
      u_is_limited_speed_active.real = this->is_limited_speed_active;
      *(outbuffer + offset + 0) = (u_is_limited_speed_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_limited_speed_active);
      union {
        bool real;
        uint8_t base;
      } u_in_sleep_mode;
      u_in_sleep_mode.real = this->in_sleep_mode;
      *(outbuffer + offset + 0) = (u_in_sleep_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->in_sleep_mode);
      union {
        bool real;
        uint8_t base;
      } u_in_manual_mode;
      u_in_manual_mode.real = this->in_manual_mode;
      *(outbuffer + offset + 0) = (u_in_manual_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->in_manual_mode);
      union {
        bool real;
        uint8_t base;
      } u_is_manual_mode_restart_required;
      u_is_manual_mode_restart_required.real = this->is_manual_mode_restart_required;
      *(outbuffer + offset + 0) = (u_is_manual_mode_restart_required.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_manual_mode_restart_required);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_connected;
      u_is_connected.base = 0;
      u_is_connected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_connected = u_is_connected.real;
      offset += sizeof(this->is_connected);
      union {
        bool real;
        uint8_t base;
      } u_is_firmware_ok;
      u_is_firmware_ok.base = 0;
      u_is_firmware_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_firmware_ok = u_is_firmware_ok.real;
      offset += sizeof(this->is_firmware_ok);
      union {
        int32_t real;
        uint32_t base;
      } u_firmware_version;
      u_firmware_version.base = 0;
      u_firmware_version.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_firmware_version.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_firmware_version.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_firmware_version.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->firmware_version = u_firmware_version.real;
      offset += sizeof(this->firmware_version);
      union {
        bool real;
        uint8_t base;
      } u_in_protective_stop;
      u_in_protective_stop.base = 0;
      u_in_protective_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->in_protective_stop = u_in_protective_stop.real;
      offset += sizeof(this->in_protective_stop);
      union {
        bool real;
        uint8_t base;
      } u_in_emergency_stop;
      u_in_emergency_stop.base = 0;
      u_in_emergency_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->in_emergency_stop = u_in_emergency_stop.real;
      offset += sizeof(this->in_emergency_stop);
      union {
        bool real;
        uint8_t base;
      } u_sto_feedback;
      u_sto_feedback.base = 0;
      u_sto_feedback.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sto_feedback = u_sto_feedback.real;
      offset += sizeof(this->sto_feedback);
      union {
        bool real;
        uint8_t base;
      } u_is_restart_required;
      u_is_restart_required.base = 0;
      u_is_restart_required.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_restart_required = u_is_restart_required.real;
      offset += sizeof(this->is_restart_required);
      union {
        bool real;
        uint8_t base;
      } u_is_safety_muted;
      u_is_safety_muted.base = 0;
      u_is_safety_muted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_safety_muted = u_is_safety_muted.real;
      offset += sizeof(this->is_safety_muted);
      union {
        bool real;
        uint8_t base;
      } u_is_limited_speed_active;
      u_is_limited_speed_active.base = 0;
      u_is_limited_speed_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_limited_speed_active = u_is_limited_speed_active.real;
      offset += sizeof(this->is_limited_speed_active);
      union {
        bool real;
        uint8_t base;
      } u_in_sleep_mode;
      u_in_sleep_mode.base = 0;
      u_in_sleep_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->in_sleep_mode = u_in_sleep_mode.real;
      offset += sizeof(this->in_sleep_mode);
      union {
        bool real;
        uint8_t base;
      } u_in_manual_mode;
      u_in_manual_mode.base = 0;
      u_in_manual_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->in_manual_mode = u_in_manual_mode.real;
      offset += sizeof(this->in_manual_mode);
      union {
        bool real;
        uint8_t base;
      } u_is_manual_mode_restart_required;
      u_is_manual_mode_restart_required.base = 0;
      u_is_manual_mode_restart_required.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_manual_mode_restart_required = u_is_manual_mode_restart_required.real;
      offset += sizeof(this->is_manual_mode_restart_required);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/SafetyStatus"; };
    virtual const char * getMD5() override { return "785ce55b5098efd15a400a9dabaf908f"; };

  };

}
#endif
