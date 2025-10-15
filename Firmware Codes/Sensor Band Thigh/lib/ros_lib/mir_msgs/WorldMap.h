#ifndef _ROS_mir_msgs_WorldMap_h
#define _ROS_mir_msgs_WorldMap_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mir_msgs/ResourcesState.h"
#include "mir_msgs/ExternalRobots.h"

namespace mir_msgs
{

  class WorldMap : public ros::Msg
  {
    public:
      typedef mir_msgs::ResourcesState _positions_type;
      _positions_type positions;
      typedef mir_msgs::ResourcesState _areas_type;
      _areas_type areas;
      typedef mir_msgs::ExternalRobots _robots_type;
      _robots_type robots;
      typedef int32_t _map_id_type;
      _map_id_type map_id;

    WorldMap():
      positions(),
      areas(),
      robots(),
      map_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->positions.serialize(outbuffer + offset);
      offset += this->areas.serialize(outbuffer + offset);
      offset += this->robots.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_map_id;
      u_map_id.real = this->map_id;
      *(outbuffer + offset + 0) = (u_map_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_map_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_map_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_map_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->map_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->positions.deserialize(inbuffer + offset);
      offset += this->areas.deserialize(inbuffer + offset);
      offset += this->robots.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_map_id;
      u_map_id.base = 0;
      u_map_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_map_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_map_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_map_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->map_id = u_map_id.real;
      offset += sizeof(this->map_id);
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/WorldMap"; };
    virtual const char * getMD5() override { return "aa59ba608dd9e6832f265f7913fdaa3b"; };

  };

}
#endif
