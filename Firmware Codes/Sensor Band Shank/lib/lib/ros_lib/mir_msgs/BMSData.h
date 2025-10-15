#ifndef _ROS_mir_msgs_BMSData_h
#define _ROS_mir_msgs_BMSData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class BMSData : public ros::Msg
  {
    public:
      typedef float _pack_voltage_type;
      _pack_voltage_type pack_voltage;
      typedef float _charge_current_type;
      _charge_current_type charge_current;
      typedef float _discharge_current_type;
      _discharge_current_type discharge_current;
      typedef int32_t _state_of_charge_type;
      _state_of_charge_type state_of_charge;
      typedef float _remaining_time_to_full_charge_type;
      _remaining_time_to_full_charge_type remaining_time_to_full_charge;
      typedef int32_t _remaining_capacity_type;
      _remaining_capacity_type remaining_capacity;
      typedef int32_t _state_of_health_type;
      _state_of_health_type state_of_health;
      typedef int32_t _status_flags_type;
      _status_flags_type status_flags;
      typedef int32_t _temperature_type;
      _temperature_type temperature;
      uint32_t cell_voltage_length;
      typedef uint32_t _cell_voltage_type;
      _cell_voltage_type st_cell_voltage;
      _cell_voltage_type * cell_voltage;
      typedef float _last_battery_msg_time_type;
      _last_battery_msg_time_type last_battery_msg_time;
      enum { DISCHARGING = 1 };
      enum { CHARGING = 2 };

    BMSData():
      pack_voltage(0),
      charge_current(0),
      discharge_current(0),
      state_of_charge(0),
      remaining_time_to_full_charge(0),
      remaining_capacity(0),
      state_of_health(0),
      status_flags(0),
      temperature(0),
      cell_voltage_length(0), st_cell_voltage(), cell_voltage(nullptr),
      last_battery_msg_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->pack_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->charge_current);
      offset += serializeAvrFloat64(outbuffer + offset, this->discharge_current);
      union {
        int32_t real;
        uint32_t base;
      } u_state_of_charge;
      u_state_of_charge.real = this->state_of_charge;
      *(outbuffer + offset + 0) = (u_state_of_charge.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state_of_charge.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_state_of_charge.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_state_of_charge.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state_of_charge);
      offset += serializeAvrFloat64(outbuffer + offset, this->remaining_time_to_full_charge);
      union {
        int32_t real;
        uint32_t base;
      } u_remaining_capacity;
      u_remaining_capacity.real = this->remaining_capacity;
      *(outbuffer + offset + 0) = (u_remaining_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_remaining_capacity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_remaining_capacity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_remaining_capacity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->remaining_capacity);
      union {
        int32_t real;
        uint32_t base;
      } u_state_of_health;
      u_state_of_health.real = this->state_of_health;
      *(outbuffer + offset + 0) = (u_state_of_health.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_state_of_health.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_state_of_health.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_state_of_health.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state_of_health);
      union {
        int32_t real;
        uint32_t base;
      } u_status_flags;
      u_status_flags.real = this->status_flags;
      *(outbuffer + offset + 0) = (u_status_flags.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status_flags.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_status_flags.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_status_flags.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status_flags);
      union {
        int32_t real;
        uint32_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature);
      *(outbuffer + offset + 0) = (this->cell_voltage_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cell_voltage_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cell_voltage_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cell_voltage_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_voltage_length);
      for( uint32_t i = 0; i < cell_voltage_length; i++){
      *(outbuffer + offset + 0) = (this->cell_voltage[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cell_voltage[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cell_voltage[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cell_voltage[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_voltage[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->last_battery_msg_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pack_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->charge_current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->discharge_current));
      union {
        int32_t real;
        uint32_t base;
      } u_state_of_charge;
      u_state_of_charge.base = 0;
      u_state_of_charge.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state_of_charge.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_state_of_charge.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_state_of_charge.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->state_of_charge = u_state_of_charge.real;
      offset += sizeof(this->state_of_charge);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->remaining_time_to_full_charge));
      union {
        int32_t real;
        uint32_t base;
      } u_remaining_capacity;
      u_remaining_capacity.base = 0;
      u_remaining_capacity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_remaining_capacity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_remaining_capacity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_remaining_capacity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->remaining_capacity = u_remaining_capacity.real;
      offset += sizeof(this->remaining_capacity);
      union {
        int32_t real;
        uint32_t base;
      } u_state_of_health;
      u_state_of_health.base = 0;
      u_state_of_health.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_state_of_health.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_state_of_health.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_state_of_health.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->state_of_health = u_state_of_health.real;
      offset += sizeof(this->state_of_health);
      union {
        int32_t real;
        uint32_t base;
      } u_status_flags;
      u_status_flags.base = 0;
      u_status_flags.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status_flags.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_status_flags.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_status_flags.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->status_flags = u_status_flags.real;
      offset += sizeof(this->status_flags);
      union {
        int32_t real;
        uint32_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
      uint32_t cell_voltage_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cell_voltage_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cell_voltage_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cell_voltage_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cell_voltage_length);
      if(cell_voltage_lengthT > cell_voltage_length)
        this->cell_voltage = (uint32_t*)realloc(this->cell_voltage, cell_voltage_lengthT * sizeof(uint32_t));
      cell_voltage_length = cell_voltage_lengthT;
      for( uint32_t i = 0; i < cell_voltage_length; i++){
      this->st_cell_voltage =  ((uint32_t) (*(inbuffer + offset)));
      this->st_cell_voltage |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_cell_voltage |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_cell_voltage |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_cell_voltage);
        memcpy( &(this->cell_voltage[i]), &(this->st_cell_voltage), sizeof(uint32_t));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->last_battery_msg_time));
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/BMSData"; };
    virtual const char * getMD5() override { return "d493696478cec84b48f8cbfeb3941739"; };

  };

}
#endif
