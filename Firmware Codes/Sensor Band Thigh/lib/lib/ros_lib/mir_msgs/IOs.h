#ifndef _ROS_mir_msgs_IOs_h
#define _ROS_mir_msgs_IOs_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class IOs : public ros::Msg
  {
    public:
      typedef const char* _module_guid_type;
      _module_guid_type module_guid;
      typedef bool _connected_type;
      _connected_type connected;
      typedef uint8_t _status_type;
      _status_type status;
      typedef int8_t _num_inputs_type;
      _num_inputs_type num_inputs;
      uint32_t input_state_length;
      typedef bool _input_state_type;
      _input_state_type st_input_state;
      _input_state_type * input_state;
      typedef int8_t _num_outputs_type;
      _num_outputs_type num_outputs;
      uint32_t output_state_length;
      typedef bool _output_state_type;
      _output_state_type st_output_state;
      _output_state_type * output_state;
      typedef const char* _ip_type;
      _ip_type ip;
      typedef const char* _error_type;
      _error_type error;
      enum { DONE = 0 };
      enum { STARTED = 1 };
      enum { ERROR = 3 };

    IOs():
      module_guid(""),
      connected(0),
      status(0),
      num_inputs(0),
      input_state_length(0), st_input_state(), input_state(nullptr),
      num_outputs(0),
      output_state_length(0), st_output_state(), output_state(nullptr),
      ip(""),
      error("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_module_guid = strlen(this->module_guid);
      varToArr(outbuffer + offset, length_module_guid);
      offset += 4;
      memcpy(outbuffer + offset, this->module_guid, length_module_guid);
      offset += length_module_guid;
      union {
        bool real;
        uint8_t base;
      } u_connected;
      u_connected.real = this->connected;
      *(outbuffer + offset + 0) = (u_connected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->connected);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      union {
        int8_t real;
        uint8_t base;
      } u_num_inputs;
      u_num_inputs.real = this->num_inputs;
      *(outbuffer + offset + 0) = (u_num_inputs.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->num_inputs);
      *(outbuffer + offset + 0) = (this->input_state_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->input_state_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->input_state_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->input_state_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_state_length);
      for( uint32_t i = 0; i < input_state_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_input_statei;
      u_input_statei.real = this->input_state[i];
      *(outbuffer + offset + 0) = (u_input_statei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->input_state[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_num_outputs;
      u_num_outputs.real = this->num_outputs;
      *(outbuffer + offset + 0) = (u_num_outputs.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->num_outputs);
      *(outbuffer + offset + 0) = (this->output_state_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->output_state_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->output_state_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->output_state_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_state_length);
      for( uint32_t i = 0; i < output_state_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_output_statei;
      u_output_statei.real = this->output_state[i];
      *(outbuffer + offset + 0) = (u_output_statei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->output_state[i]);
      }
      uint32_t length_ip = strlen(this->ip);
      varToArr(outbuffer + offset, length_ip);
      offset += 4;
      memcpy(outbuffer + offset, this->ip, length_ip);
      offset += length_ip;
      uint32_t length_error = strlen(this->error);
      varToArr(outbuffer + offset, length_error);
      offset += 4;
      memcpy(outbuffer + offset, this->error, length_error);
      offset += length_error;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_module_guid;
      arrToVar(length_module_guid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_module_guid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_module_guid-1]=0;
      this->module_guid = (char *)(inbuffer + offset-1);
      offset += length_module_guid;
      union {
        bool real;
        uint8_t base;
      } u_connected;
      u_connected.base = 0;
      u_connected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->connected = u_connected.real;
      offset += sizeof(this->connected);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      union {
        int8_t real;
        uint8_t base;
      } u_num_inputs;
      u_num_inputs.base = 0;
      u_num_inputs.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->num_inputs = u_num_inputs.real;
      offset += sizeof(this->num_inputs);
      uint32_t input_state_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      input_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      input_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      input_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->input_state_length);
      if(input_state_lengthT > input_state_length)
        this->input_state = (bool*)realloc(this->input_state, input_state_lengthT * sizeof(bool));
      input_state_length = input_state_lengthT;
      for( uint32_t i = 0; i < input_state_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_input_state;
      u_st_input_state.base = 0;
      u_st_input_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_input_state = u_st_input_state.real;
      offset += sizeof(this->st_input_state);
        memcpy( &(this->input_state[i]), &(this->st_input_state), sizeof(bool));
      }
      union {
        int8_t real;
        uint8_t base;
      } u_num_outputs;
      u_num_outputs.base = 0;
      u_num_outputs.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->num_outputs = u_num_outputs.real;
      offset += sizeof(this->num_outputs);
      uint32_t output_state_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      output_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      output_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      output_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->output_state_length);
      if(output_state_lengthT > output_state_length)
        this->output_state = (bool*)realloc(this->output_state, output_state_lengthT * sizeof(bool));
      output_state_length = output_state_lengthT;
      for( uint32_t i = 0; i < output_state_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_output_state;
      u_st_output_state.base = 0;
      u_st_output_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_output_state = u_st_output_state.real;
      offset += sizeof(this->st_output_state);
        memcpy( &(this->output_state[i]), &(this->st_output_state), sizeof(bool));
      }
      uint32_t length_ip;
      arrToVar(length_ip, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ip; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ip-1]=0;
      this->ip = (char *)(inbuffer + offset-1);
      offset += length_ip;
      uint32_t length_error;
      arrToVar(length_error, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_error; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_error-1]=0;
      this->error = (char *)(inbuffer + offset-1);
      offset += length_error;
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/IOs"; };
    virtual const char * getMD5() override { return "6266405913b096bf8e69b775d090b781"; };

  };

}
#endif
