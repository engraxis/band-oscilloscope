#ifndef _ROS_mir_msgs_ResourcesState_h
#define _ROS_mir_msgs_ResourcesState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mir_msgs/ResourceState.h"

namespace mir_msgs
{

  class ResourcesState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t resources_length;
      typedef mir_msgs::ResourceState _resources_type;
      _resources_type st_resources;
      _resources_type * resources;

    ResourcesState():
      header(),
      resources_length(0), st_resources(), resources(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->resources_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->resources_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->resources_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->resources_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->resources_length);
      for( uint32_t i = 0; i < resources_length; i++){
      offset += this->resources[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t resources_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      resources_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      resources_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      resources_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->resources_length);
      if(resources_lengthT > resources_length)
        this->resources = (mir_msgs::ResourceState*)realloc(this->resources, resources_lengthT * sizeof(mir_msgs::ResourceState));
      resources_length = resources_lengthT;
      for( uint32_t i = 0; i < resources_length; i++){
      offset += this->st_resources.deserialize(inbuffer + offset);
        memcpy( &(this->resources[i]), &(this->st_resources), sizeof(mir_msgs::ResourceState));
      }
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/ResourcesState"; };
    virtual const char * getMD5() override { return "2e0263c09697d545680f9e07fbd8dd7e"; };

  };

}
#endif
