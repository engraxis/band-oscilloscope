#ifndef _ROS_mir_msgs_ExternalRobots_h
#define _ROS_mir_msgs_ExternalRobots_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mir_msgs/ExternalRobot.h"

namespace mir_msgs
{

  class ExternalRobots : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t robots_length;
      typedef mir_msgs::ExternalRobot _robots_type;
      _robots_type st_robots;
      _robots_type * robots;

    ExternalRobots():
      header(),
      robots_length(0), st_robots(), robots(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->robots_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->robots_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->robots_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->robots_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->robots_length);
      for( uint32_t i = 0; i < robots_length; i++){
      offset += this->robots[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t robots_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      robots_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      robots_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      robots_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->robots_length);
      if(robots_lengthT > robots_length)
        this->robots = (mir_msgs::ExternalRobot*)realloc(this->robots, robots_lengthT * sizeof(mir_msgs::ExternalRobot));
      robots_length = robots_lengthT;
      for( uint32_t i = 0; i < robots_length; i++){
      offset += this->st_robots.deserialize(inbuffer + offset);
        memcpy( &(this->robots[i]), &(this->st_robots), sizeof(mir_msgs::ExternalRobot));
      }
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/ExternalRobots"; };
    virtual const char * getMD5() override { return "7ba6f379de921749221a980f7a1a21f4"; };

  };

}
#endif
