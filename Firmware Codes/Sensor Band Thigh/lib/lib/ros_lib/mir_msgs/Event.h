#ifndef _ROS_mir_msgs_Event_h
#define _ROS_mir_msgs_Event_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace mir_msgs
{

  class Event : public ros::Msg
  {
    public:
      typedef uint32_t _eventType_type;
      _eventType_type eventType;
      typedef const char* _area_guid_type;
      _area_guid_type area_guid;
      typedef const char* _area_name_type;
      _area_name_type area_name;
      uint32_t polygon_length;
      typedef geometry_msgs::Point _polygon_type;
      _polygon_type st_polygon;
      _polygon_type * polygon;
      enum { EV_SPEED = 1 };
      enum { EV_BLINK = 2 };
      enum { EV_SOUND = 3 };
      enum { EV_DOOR = 4 };
      enum { EV_AMCLOFF = 5 };
      enum { EV_FWDDIST = 6 };
      enum { EV_IO = 7 };
      enum { EV_FLEETLCK = 8	 };
      enum { EV_EMERGENCY = 9	 };

    Event():
      eventType(0),
      area_guid(""),
      area_name(""),
      polygon_length(0), st_polygon(), polygon(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->eventType >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->eventType >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->eventType >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->eventType >> (8 * 3)) & 0xFF;
      offset += sizeof(this->eventType);
      uint32_t length_area_guid = strlen(this->area_guid);
      varToArr(outbuffer + offset, length_area_guid);
      offset += 4;
      memcpy(outbuffer + offset, this->area_guid, length_area_guid);
      offset += length_area_guid;
      uint32_t length_area_name = strlen(this->area_name);
      varToArr(outbuffer + offset, length_area_name);
      offset += 4;
      memcpy(outbuffer + offset, this->area_name, length_area_name);
      offset += length_area_name;
      *(outbuffer + offset + 0) = (this->polygon_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->polygon_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->polygon_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->polygon_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->polygon_length);
      for( uint32_t i = 0; i < polygon_length; i++){
      offset += this->polygon[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->eventType =  ((uint32_t) (*(inbuffer + offset)));
      this->eventType |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->eventType |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->eventType |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->eventType);
      uint32_t length_area_guid;
      arrToVar(length_area_guid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_area_guid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_area_guid-1]=0;
      this->area_guid = (char *)(inbuffer + offset-1);
      offset += length_area_guid;
      uint32_t length_area_name;
      arrToVar(length_area_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_area_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_area_name-1]=0;
      this->area_name = (char *)(inbuffer + offset-1);
      offset += length_area_name;
      uint32_t polygon_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      polygon_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      polygon_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      polygon_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->polygon_length);
      if(polygon_lengthT > polygon_length)
        this->polygon = (geometry_msgs::Point*)realloc(this->polygon, polygon_lengthT * sizeof(geometry_msgs::Point));
      polygon_length = polygon_lengthT;
      for( uint32_t i = 0; i < polygon_length; i++){
      offset += this->st_polygon.deserialize(inbuffer + offset);
        memcpy( &(this->polygon[i]), &(this->st_polygon), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/Event"; };
    virtual const char * getMD5() override { return "03782c584d14555433c75de52c9adea7"; };

  };

}
#endif
