#ifndef _ROS_mir_msgs_RobotStatus_h
#define _ROS_mir_msgs_RobotStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mir_msgs/Error.h"
#include "mir_msgs/HookStatus.h"
#include "mir_msgs/Pose2D.h"
#include "mir_msgs/Twist2D.h"
#include "mir_msgs/UserPrompt.h"

namespace mir_msgs
{

  class RobotStatus : public ros::Msg
  {
    public:
      typedef float _battery_percentage_type;
      _battery_percentage_type battery_percentage;
      typedef int32_t _battery_time_remaining_type;
      _battery_time_remaining_type battery_time_remaining;
      typedef float _battery_voltage_type;
      _battery_voltage_type battery_voltage;
      typedef float _distance_to_next_target_type;
      _distance_to_next_target_type distance_to_next_target;
      uint32_t errors_length;
      typedef mir_msgs::Error _errors_type;
      _errors_type st_errors;
      _errors_type * errors;
      typedef const char* _footprint_type;
      _footprint_type footprint;
      typedef mir_msgs::HookStatus _hook_status_type;
      _hook_status_type hook_status;
      typedef const char* _map_id_type;
      _map_id_type map_id;
      typedef bool _unloaded_map_changes_type;
      _unloaded_map_changes_type unloaded_map_changes;
      typedef int32_t _mission_queue_id_type;
      _mission_queue_id_type mission_queue_id;
      typedef const char* _mission_text_type;
      _mission_text_type mission_text;
      typedef int32_t _mode_id_type;
      _mode_id_type mode_id;
      typedef const char* _mode_text_type;
      _mode_text_type mode_text;
      typedef float _moved_type;
      _moved_type moved;
      typedef mir_msgs::Pose2D _position_type;
      _position_type position;
      typedef const char* _robot_name_type;
      _robot_name_type robot_name;
      typedef const char* _session_id_type;
      _session_id_type session_id;
      typedef const char* _software_version_type;
      _software_version_type software_version;
      typedef uint8_t _state_id_type;
      _state_id_type state_id;
      typedef const char* _state_text_type;
      _state_text_type state_text;
      typedef int32_t _uptime_type;
      _uptime_type uptime;
      typedef mir_msgs::Twist2D _velocity_type;
      _velocity_type velocity;
      typedef mir_msgs::UserPrompt _user_prompt_type;
      _user_prompt_type user_prompt;

    RobotStatus():
      battery_percentage(0),
      battery_time_remaining(0),
      battery_voltage(0),
      distance_to_next_target(0),
      errors_length(0), st_errors(), errors(nullptr),
      footprint(""),
      hook_status(),
      map_id(""),
      unloaded_map_changes(0),
      mission_queue_id(0),
      mission_text(""),
      mode_id(0),
      mode_text(""),
      moved(0),
      position(),
      robot_name(""),
      session_id(""),
      software_version(""),
      state_id(0),
      state_text(""),
      uptime(0),
      velocity(),
      user_prompt()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
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
      } u_battery_time_remaining;
      u_battery_time_remaining.real = this->battery_time_remaining;
      *(outbuffer + offset + 0) = (u_battery_time_remaining.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_time_remaining.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_time_remaining.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_time_remaining.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_time_remaining);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_distance_to_next_target;
      u_distance_to_next_target.real = this->distance_to_next_target;
      *(outbuffer + offset + 0) = (u_distance_to_next_target.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance_to_next_target.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance_to_next_target.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance_to_next_target.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance_to_next_target);
      *(outbuffer + offset + 0) = (this->errors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->errors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->errors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->errors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->errors_length);
      for( uint32_t i = 0; i < errors_length; i++){
      offset += this->errors[i].serialize(outbuffer + offset);
      }
      uint32_t length_footprint = strlen(this->footprint);
      varToArr(outbuffer + offset, length_footprint);
      offset += 4;
      memcpy(outbuffer + offset, this->footprint, length_footprint);
      offset += length_footprint;
      offset += this->hook_status.serialize(outbuffer + offset);
      uint32_t length_map_id = strlen(this->map_id);
      varToArr(outbuffer + offset, length_map_id);
      offset += 4;
      memcpy(outbuffer + offset, this->map_id, length_map_id);
      offset += length_map_id;
      union {
        bool real;
        uint8_t base;
      } u_unloaded_map_changes;
      u_unloaded_map_changes.real = this->unloaded_map_changes;
      *(outbuffer + offset + 0) = (u_unloaded_map_changes.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->unloaded_map_changes);
      union {
        int32_t real;
        uint32_t base;
      } u_mission_queue_id;
      u_mission_queue_id.real = this->mission_queue_id;
      *(outbuffer + offset + 0) = (u_mission_queue_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mission_queue_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mission_queue_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mission_queue_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mission_queue_id);
      uint32_t length_mission_text = strlen(this->mission_text);
      varToArr(outbuffer + offset, length_mission_text);
      offset += 4;
      memcpy(outbuffer + offset, this->mission_text, length_mission_text);
      offset += length_mission_text;
      union {
        int32_t real;
        uint32_t base;
      } u_mode_id;
      u_mode_id.real = this->mode_id;
      *(outbuffer + offset + 0) = (u_mode_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mode_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mode_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mode_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mode_id);
      uint32_t length_mode_text = strlen(this->mode_text);
      varToArr(outbuffer + offset, length_mode_text);
      offset += 4;
      memcpy(outbuffer + offset, this->mode_text, length_mode_text);
      offset += length_mode_text;
      offset += serializeAvrFloat64(outbuffer + offset, this->moved);
      offset += this->position.serialize(outbuffer + offset);
      uint32_t length_robot_name = strlen(this->robot_name);
      varToArr(outbuffer + offset, length_robot_name);
      offset += 4;
      memcpy(outbuffer + offset, this->robot_name, length_robot_name);
      offset += length_robot_name;
      uint32_t length_session_id = strlen(this->session_id);
      varToArr(outbuffer + offset, length_session_id);
      offset += 4;
      memcpy(outbuffer + offset, this->session_id, length_session_id);
      offset += length_session_id;
      uint32_t length_software_version = strlen(this->software_version);
      varToArr(outbuffer + offset, length_software_version);
      offset += 4;
      memcpy(outbuffer + offset, this->software_version, length_software_version);
      offset += length_software_version;
      *(outbuffer + offset + 0) = (this->state_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state_id);
      uint32_t length_state_text = strlen(this->state_text);
      varToArr(outbuffer + offset, length_state_text);
      offset += 4;
      memcpy(outbuffer + offset, this->state_text, length_state_text);
      offset += length_state_text;
      union {
        int32_t real;
        uint32_t base;
      } u_uptime;
      u_uptime.real = this->uptime;
      *(outbuffer + offset + 0) = (u_uptime.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_uptime.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_uptime.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_uptime.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->uptime);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->user_prompt.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
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
      } u_battery_time_remaining;
      u_battery_time_remaining.base = 0;
      u_battery_time_remaining.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_time_remaining.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_time_remaining.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_time_remaining.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_time_remaining = u_battery_time_remaining.real;
      offset += sizeof(this->battery_time_remaining);
      union {
        float real;
        uint32_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
      union {
        float real;
        uint32_t base;
      } u_distance_to_next_target;
      u_distance_to_next_target.base = 0;
      u_distance_to_next_target.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance_to_next_target.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance_to_next_target.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance_to_next_target.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance_to_next_target = u_distance_to_next_target.real;
      offset += sizeof(this->distance_to_next_target);
      uint32_t errors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->errors_length);
      if(errors_lengthT > errors_length)
        this->errors = (mir_msgs::Error*)realloc(this->errors, errors_lengthT * sizeof(mir_msgs::Error));
      errors_length = errors_lengthT;
      for( uint32_t i = 0; i < errors_length; i++){
      offset += this->st_errors.deserialize(inbuffer + offset);
        memcpy( &(this->errors[i]), &(this->st_errors), sizeof(mir_msgs::Error));
      }
      uint32_t length_footprint;
      arrToVar(length_footprint, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_footprint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_footprint-1]=0;
      this->footprint = (char *)(inbuffer + offset-1);
      offset += length_footprint;
      offset += this->hook_status.deserialize(inbuffer + offset);
      uint32_t length_map_id;
      arrToVar(length_map_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_map_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_map_id-1]=0;
      this->map_id = (char *)(inbuffer + offset-1);
      offset += length_map_id;
      union {
        bool real;
        uint8_t base;
      } u_unloaded_map_changes;
      u_unloaded_map_changes.base = 0;
      u_unloaded_map_changes.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->unloaded_map_changes = u_unloaded_map_changes.real;
      offset += sizeof(this->unloaded_map_changes);
      union {
        int32_t real;
        uint32_t base;
      } u_mission_queue_id;
      u_mission_queue_id.base = 0;
      u_mission_queue_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mission_queue_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mission_queue_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mission_queue_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mission_queue_id = u_mission_queue_id.real;
      offset += sizeof(this->mission_queue_id);
      uint32_t length_mission_text;
      arrToVar(length_mission_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mission_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mission_text-1]=0;
      this->mission_text = (char *)(inbuffer + offset-1);
      offset += length_mission_text;
      union {
        int32_t real;
        uint32_t base;
      } u_mode_id;
      u_mode_id.base = 0;
      u_mode_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mode_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mode_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mode_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mode_id = u_mode_id.real;
      offset += sizeof(this->mode_id);
      uint32_t length_mode_text;
      arrToVar(length_mode_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode_text-1]=0;
      this->mode_text = (char *)(inbuffer + offset-1);
      offset += length_mode_text;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->moved));
      offset += this->position.deserialize(inbuffer + offset);
      uint32_t length_robot_name;
      arrToVar(length_robot_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_robot_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_robot_name-1]=0;
      this->robot_name = (char *)(inbuffer + offset-1);
      offset += length_robot_name;
      uint32_t length_session_id;
      arrToVar(length_session_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_session_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_session_id-1]=0;
      this->session_id = (char *)(inbuffer + offset-1);
      offset += length_session_id;
      uint32_t length_software_version;
      arrToVar(length_software_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_software_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_software_version-1]=0;
      this->software_version = (char *)(inbuffer + offset-1);
      offset += length_software_version;
      this->state_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state_id);
      uint32_t length_state_text;
      arrToVar(length_state_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state_text-1]=0;
      this->state_text = (char *)(inbuffer + offset-1);
      offset += length_state_text;
      union {
        int32_t real;
        uint32_t base;
      } u_uptime;
      u_uptime.base = 0;
      u_uptime.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_uptime.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_uptime.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_uptime.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->uptime = u_uptime.real;
      offset += sizeof(this->uptime);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->user_prompt.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/RobotStatus"; };
    virtual const char * getMD5() override { return "18d07d8b4603caa865c6e7d49636c2d6"; };

  };

}
#endif
