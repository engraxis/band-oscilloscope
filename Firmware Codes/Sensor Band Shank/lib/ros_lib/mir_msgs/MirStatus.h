#ifndef _ROS_mir_msgs_MirStatus_h
#define _ROS_mir_msgs_MirStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class MirStatus : public ros::Msg
  {
    public:
      typedef int32_t _state_type;
      _state_type state;
      typedef const char* _mode_type;
      _mode_type mode;
      typedef const char* _msg_type;
      _msg_type msg;
      typedef float _uptime_type;
      _uptime_type uptime;
      typedef float _moved_type;
      _moved_type moved;
      typedef float _battery_type;
      _battery_type battery;
      typedef float _battery_percentage_type;
      _battery_percentage_type battery_percentage;
      typedef int32_t _battery_time_left_type;
      _battery_time_left_type battery_time_left;
      typedef float _eta_type;
      _eta_type eta;

    MirStatus():
      state(0),
      mode(""),
      msg(""),
      uptime(0),
      moved(0),
      battery(0),
      battery_percentage(0),
      battery_time_left(0),
      eta(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_state.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state);
      uint32_t length_mode = strlen(this->mode);
      varToArr(outbuffer + offset, length_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      uint32_t length_msg = strlen(this->msg);
      varToArr(outbuffer + offset, length_msg);
      offset += 4;
      memcpy(outbuffer + offset, this->msg, length_msg);
      offset += length_msg;
      union {
        float real;
        uint32_t base;
      } u_uptime;
      u_uptime.real = this->uptime;
      *(outbuffer + offset + 0) = (u_uptime.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_uptime.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_uptime.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_uptime.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->uptime);
      union {
        float real;
        uint32_t base;
      } u_moved;
      u_moved.real = this->moved;
      *(outbuffer + offset + 0) = (u_moved.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_moved.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_moved.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_moved.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->moved);
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.real = this->battery;
      *(outbuffer + offset + 0) = (u_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery);
      union {
        float real;
        uint32_t base;
      } u_battery_percentage;
      u_battery_percentage.real = this->battery_percentage;
      *(outbuffer + offset + 0) = (u_battery_percentage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_percentage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_percentage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_percentage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_percentage);
      union {
        int32_t real;
        uint32_t base;
      } u_battery_time_left;
      u_battery_time_left.real = this->battery_time_left;
      *(outbuffer + offset + 0) = (u_battery_time_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_time_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_time_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_time_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_time_left);
      union {
        float real;
        uint32_t base;
      } u_eta;
      u_eta.real = this->eta;
      *(outbuffer + offset + 0) = (u_eta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_eta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_eta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_eta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->eta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_state.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->state = u_state.real;
      offset += sizeof(this->state);
      uint32_t length_mode;
      arrToVar(length_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
      uint32_t length_msg;
      arrToVar(length_msg, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_msg-1]=0;
      this->msg = (char *)(inbuffer + offset-1);
      offset += length_msg;
      union {
        float real;
        uint32_t base;
      } u_uptime;
      u_uptime.base = 0;
      u_uptime.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_uptime.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_uptime.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_uptime.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->uptime = u_uptime.real;
      offset += sizeof(this->uptime);
      union {
        float real;
        uint32_t base;
      } u_moved;
      u_moved.base = 0;
      u_moved.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_moved.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_moved.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_moved.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->moved = u_moved.real;
      offset += sizeof(this->moved);
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.base = 0;
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery = u_battery.real;
      offset += sizeof(this->battery);
      union {
        float real;
        uint32_t base;
      } u_battery_percentage;
      u_battery_percentage.base = 0;
      u_battery_percentage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_percentage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_percentage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_percentage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_percentage = u_battery_percentage.real;
      offset += sizeof(this->battery_percentage);
      union {
        int32_t real;
        uint32_t base;
      } u_battery_time_left;
      u_battery_time_left.base = 0;
      u_battery_time_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_time_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_time_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_time_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_time_left = u_battery_time_left.real;
      offset += sizeof(this->battery_time_left);
      union {
        float real;
        uint32_t base;
      } u_eta;
      u_eta.base = 0;
      u_eta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_eta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_eta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_eta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->eta = u_eta.real;
      offset += sizeof(this->eta);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/MirStatus"; };
    virtual const char * getMD5() override { return "67e1f2c3710bfe1130e859b6c4f4e0c2"; };

  };

}
#endif
