#ifndef _ROS_mir_msgs_UserPrompt_h
#define _ROS_mir_msgs_UserPrompt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/duration.h"

namespace mir_msgs
{

  class UserPrompt : public ros::Msg
  {
    public:
      typedef bool _has_request_type;
      _has_request_type has_request;
      typedef const char* _guid_type;
      _guid_type guid;
      typedef const char* _user_group_type;
      _user_group_type user_group;
      typedef const char* _question_type;
      _question_type question;
      uint32_t options_length;
      typedef char* _options_type;
      _options_type st_options;
      _options_type * options;
      typedef ros::Duration _timeout_type;
      _timeout_type timeout;

    UserPrompt():
      has_request(0),
      guid(""),
      user_group(""),
      question(""),
      options_length(0), st_options(), options(nullptr),
      timeout()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_has_request;
      u_has_request.real = this->has_request;
      *(outbuffer + offset + 0) = (u_has_request.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->has_request);
      uint32_t length_guid = strlen(this->guid);
      varToArr(outbuffer + offset, length_guid);
      offset += 4;
      memcpy(outbuffer + offset, this->guid, length_guid);
      offset += length_guid;
      uint32_t length_user_group = strlen(this->user_group);
      varToArr(outbuffer + offset, length_user_group);
      offset += 4;
      memcpy(outbuffer + offset, this->user_group, length_user_group);
      offset += length_user_group;
      uint32_t length_question = strlen(this->question);
      varToArr(outbuffer + offset, length_question);
      offset += 4;
      memcpy(outbuffer + offset, this->question, length_question);
      offset += length_question;
      *(outbuffer + offset + 0) = (this->options_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->options_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->options_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->options_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->options_length);
      for( uint32_t i = 0; i < options_length; i++){
      uint32_t length_optionsi = strlen(this->options[i]);
      varToArr(outbuffer + offset, length_optionsi);
      offset += 4;
      memcpy(outbuffer + offset, this->options[i], length_optionsi);
      offset += length_optionsi;
      }
      *(outbuffer + offset + 0) = (this->timeout.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeout.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeout.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeout.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeout.sec);
      *(outbuffer + offset + 0) = (this->timeout.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeout.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeout.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeout.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeout.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_has_request;
      u_has_request.base = 0;
      u_has_request.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->has_request = u_has_request.real;
      offset += sizeof(this->has_request);
      uint32_t length_guid;
      arrToVar(length_guid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_guid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_guid-1]=0;
      this->guid = (char *)(inbuffer + offset-1);
      offset += length_guid;
      uint32_t length_user_group;
      arrToVar(length_user_group, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_user_group; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_user_group-1]=0;
      this->user_group = (char *)(inbuffer + offset-1);
      offset += length_user_group;
      uint32_t length_question;
      arrToVar(length_question, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_question; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_question-1]=0;
      this->question = (char *)(inbuffer + offset-1);
      offset += length_question;
      uint32_t options_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      options_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      options_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      options_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->options_length);
      if(options_lengthT > options_length)
        this->options = (char**)realloc(this->options, options_lengthT * sizeof(char*));
      options_length = options_lengthT;
      for( uint32_t i = 0; i < options_length; i++){
      uint32_t length_st_options;
      arrToVar(length_st_options, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_options; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_options-1]=0;
      this->st_options = (char *)(inbuffer + offset-1);
      offset += length_st_options;
        memcpy( &(this->options[i]), &(this->st_options), sizeof(char*));
      }
      this->timeout.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timeout.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeout.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeout.sec);
      this->timeout.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeout.nsec);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/UserPrompt"; };
    virtual const char * getMD5() override { return "731624029b0041f5bffe8cc3d3ed3abe"; };

  };

}
#endif
