#ifndef _ROS_mir_msgs_ResourceState_h
#define _ROS_mir_msgs_ResourceState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace mir_msgs
{

  class ResourceState : public ros::Msg
  {
    public:
      uint32_t assigned_length;
      typedef char* _assigned_type;
      _assigned_type st_assigned;
      _assigned_type * assigned;
      typedef uint32_t _type_type;
      _type_type type;
      typedef uint32_t _path_idx_type;
      _path_idx_type path_idx;
      typedef float _distance_type;
      _distance_type distance;
      typedef geometry_msgs::Point _collision_point_type;
      _collision_point_type collision_point;
      uint32_t queue_length;
      typedef char* _queue_type;
      _queue_type st_queue;
      _queue_type * queue;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _guid_type;
      _guid_type guid;
      enum { ROBOT_POSITION = 0 };
      enum { STAGING_POSITION = 1 };
      enum { CHARGING_STATION = 2 };
      enum { AREA = 3 };

    ResourceState():
      assigned_length(0), st_assigned(), assigned(nullptr),
      type(0),
      path_idx(0),
      distance(0),
      collision_point(),
      queue_length(0), st_queue(), queue(nullptr),
      name(""),
      guid("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->assigned_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->assigned_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->assigned_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->assigned_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->assigned_length);
      for( uint32_t i = 0; i < assigned_length; i++){
      uint32_t length_assignedi = strlen(this->assigned[i]);
      varToArr(outbuffer + offset, length_assignedi);
      offset += 4;
      memcpy(outbuffer + offset, this->assigned[i], length_assignedi);
      offset += length_assignedi;
      }
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->type >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->type >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->type >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset + 0) = (this->path_idx >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->path_idx >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->path_idx >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->path_idx >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_idx);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      offset += this->collision_point.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->queue_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->queue_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->queue_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->queue_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->queue_length);
      for( uint32_t i = 0; i < queue_length; i++){
      uint32_t length_queuei = strlen(this->queue[i]);
      varToArr(outbuffer + offset, length_queuei);
      offset += 4;
      memcpy(outbuffer + offset, this->queue[i], length_queuei);
      offset += length_queuei;
      }
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_guid = strlen(this->guid);
      varToArr(outbuffer + offset, length_guid);
      offset += 4;
      memcpy(outbuffer + offset, this->guid, length_guid);
      offset += length_guid;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t assigned_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      assigned_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      assigned_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      assigned_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->assigned_length);
      if(assigned_lengthT > assigned_length)
        this->assigned = (char**)realloc(this->assigned, assigned_lengthT * sizeof(char*));
      assigned_length = assigned_lengthT;
      for( uint32_t i = 0; i < assigned_length; i++){
      uint32_t length_st_assigned;
      arrToVar(length_st_assigned, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_assigned; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_assigned-1]=0;
      this->st_assigned = (char *)(inbuffer + offset-1);
      offset += length_st_assigned;
        memcpy( &(this->assigned[i]), &(this->st_assigned), sizeof(char*));
      }
      this->type =  ((uint32_t) (*(inbuffer + offset)));
      this->type |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->type |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->type |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->type);
      this->path_idx =  ((uint32_t) (*(inbuffer + offset)));
      this->path_idx |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->path_idx |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->path_idx |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->path_idx);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
      offset += this->collision_point.deserialize(inbuffer + offset);
      uint32_t queue_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      queue_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      queue_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      queue_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->queue_length);
      if(queue_lengthT > queue_length)
        this->queue = (char**)realloc(this->queue, queue_lengthT * sizeof(char*));
      queue_length = queue_lengthT;
      for( uint32_t i = 0; i < queue_length; i++){
      uint32_t length_st_queue;
      arrToVar(length_st_queue, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_queue; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_queue-1]=0;
      this->st_queue = (char *)(inbuffer + offset-1);
      offset += length_st_queue;
        memcpy( &(this->queue[i]), &(this->st_queue), sizeof(char*));
      }
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_guid;
      arrToVar(length_guid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_guid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_guid-1]=0;
      this->guid = (char *)(inbuffer + offset-1);
      offset += length_guid;
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/ResourceState"; };
    virtual const char * getMD5() override { return "df6513b10e65bfe7ec44f469810ded59"; };

  };

}
#endif
