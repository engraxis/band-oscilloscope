#ifndef _ROS_mir_msgs_MissionCtrlCommand_h
#define _ROS_mir_msgs_MissionCtrlCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mir_msgs
{

  class MissionCtrlCommand : public ros::Msg
  {
    public:
      typedef const char* _description_type;
      _description_type description;
      typedef int32_t _cmd_type;
      _cmd_type cmd;
      typedef int32_t _mission_id_type;
      _mission_id_type mission_id;
      enum { CMD_GET_STATUS =  0 };
      enum { CMD_WAIT_POS_LOCK =  1 };
      enum { CMD_WAIT_AREA_LOCK =  2 };
      enum { CMD_CONTINUE =  3 };
      enum { CMD_LOAD_MISSION =  4 };

    MissionCtrlCommand():
      description(""),
      cmd(0),
      mission_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd;
      u_cmd.real = this->cmd;
      *(outbuffer + offset + 0) = (u_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd);
      union {
        int32_t real;
        uint32_t base;
      } u_mission_id;
      u_mission_id.real = this->mission_id;
      *(outbuffer + offset + 0) = (u_mission_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mission_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mission_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mission_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mission_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd;
      u_cmd.base = 0;
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd = u_cmd.real;
      offset += sizeof(this->cmd);
      union {
        int32_t real;
        uint32_t base;
      } u_mission_id;
      u_mission_id.base = 0;
      u_mission_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mission_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mission_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mission_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mission_id = u_mission_id.real;
      offset += sizeof(this->mission_id);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/MissionCtrlCommand"; };
    virtual const char * getMD5() override { return "ea52ad5a8cbcac0a68f542bb228ca82d"; };

  };

}
#endif
