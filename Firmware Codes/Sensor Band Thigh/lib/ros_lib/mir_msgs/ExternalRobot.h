#ifndef _ROS_mir_msgs_ExternalRobot_h
#define _ROS_mir_msgs_ExternalRobot_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

namespace mir_msgs
{

  class ExternalRobot : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _id_type;
      _id_type id;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _footprint_type;
      _footprint_type footprint;
      typedef const char* _ip_type;
      _ip_type ip;
      typedef uint32_t _map_id_type;
      _map_id_type map_id;
      typedef int32_t _priority_type;
      _priority_type priority;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Pose _extrapolated_pose_type;
      _extrapolated_pose_type extrapolated_pose;
      typedef geometry_msgs::Twist _twist_type;
      _twist_type twist;

    ExternalRobot():
      header(),
      id(0),
      name(""),
      footprint(""),
      ip(""),
      map_id(0),
      priority(0),
      pose(),
      extrapolated_pose(),
      twist()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_footprint = strlen(this->footprint);
      varToArr(outbuffer + offset, length_footprint);
      offset += 4;
      memcpy(outbuffer + offset, this->footprint, length_footprint);
      offset += length_footprint;
      uint32_t length_ip = strlen(this->ip);
      varToArr(outbuffer + offset, length_ip);
      offset += 4;
      memcpy(outbuffer + offset, this->ip, length_ip);
      offset += length_ip;
      *(outbuffer + offset + 0) = (this->map_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->map_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->map_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->map_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->map_id);
      union {
        int32_t real;
        uint32_t base;
      } u_priority;
      u_priority.real = this->priority;
      *(outbuffer + offset + 0) = (u_priority.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_priority.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_priority.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_priority.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->priority);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->extrapolated_pose.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_footprint;
      arrToVar(length_footprint, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_footprint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_footprint-1]=0;
      this->footprint = (char *)(inbuffer + offset-1);
      offset += length_footprint;
      uint32_t length_ip;
      arrToVar(length_ip, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ip; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ip-1]=0;
      this->ip = (char *)(inbuffer + offset-1);
      offset += length_ip;
      this->map_id =  ((uint32_t) (*(inbuffer + offset)));
      this->map_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->map_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->map_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->map_id);
      union {
        int32_t real;
        uint32_t base;
      } u_priority;
      u_priority.base = 0;
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->priority = u_priority.real;
      offset += sizeof(this->priority);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->extrapolated_pose.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/ExternalRobot"; };
    virtual const char * getMD5() override { return "2aaed3d73affd5b1b43392a37ae9a69c"; };

  };

}
#endif
