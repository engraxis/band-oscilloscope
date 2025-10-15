#ifndef _ROS_mir_msgs_WorldModel_h
#define _ROS_mir_msgs_WorldModel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mir_msgs/WorldMap.h"

namespace mir_msgs
{

  class WorldModel : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t world_map_length;
      typedef mir_msgs::WorldMap _world_map_type;
      _world_map_type st_world_map;
      _world_map_type * world_map;

    WorldModel():
      header(),
      world_map_length(0), st_world_map(), world_map(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->world_map_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->world_map_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->world_map_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->world_map_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->world_map_length);
      for( uint32_t i = 0; i < world_map_length; i++){
      offset += this->world_map[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t world_map_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      world_map_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      world_map_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      world_map_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->world_map_length);
      if(world_map_lengthT > world_map_length)
        this->world_map = (mir_msgs::WorldMap*)realloc(this->world_map, world_map_lengthT * sizeof(mir_msgs::WorldMap));
      world_map_length = world_map_lengthT;
      for( uint32_t i = 0; i < world_map_length; i++){
      offset += this->st_world_map.deserialize(inbuffer + offset);
        memcpy( &(this->world_map[i]), &(this->st_world_map), sizeof(mir_msgs::WorldMap));
      }
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/WorldModel"; };
    virtual const char * getMD5() override { return "08874a9d91b8995bca1260d0f250c218"; };

  };

}
#endif
