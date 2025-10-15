#ifndef _ROS_mir_msgs_PlanSegment_h
#define _ROS_mir_msgs_PlanSegment_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace mir_msgs
{

  class PlanSegment : public ros::Msg
  {
    public:
      typedef bool _forward_motion_type;
      _forward_motion_type forward_motion;
      typedef int32_t _start_idx_type;
      _start_idx_type start_idx;
      typedef float _length_type;
      _length_type length;
      typedef float _remaining_length_type;
      _remaining_length_type remaining_length;
      uint32_t path_length;
      typedef geometry_msgs::PoseStamped _path_type;
      _path_type st_path;
      _path_type * path;

    PlanSegment():
      forward_motion(0),
      start_idx(0),
      length(0),
      remaining_length(0),
      path_length(0), st_path(), path(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_forward_motion;
      u_forward_motion.real = this->forward_motion;
      *(outbuffer + offset + 0) = (u_forward_motion.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->forward_motion);
      union {
        int32_t real;
        uint32_t base;
      } u_start_idx;
      u_start_idx.real = this->start_idx;
      *(outbuffer + offset + 0) = (u_start_idx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_start_idx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_start_idx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_start_idx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_idx);
      offset += serializeAvrFloat64(outbuffer + offset, this->length);
      offset += serializeAvrFloat64(outbuffer + offset, this->remaining_length);
      *(outbuffer + offset + 0) = (this->path_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->path_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->path_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->path_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_length);
      for( uint32_t i = 0; i < path_length; i++){
      offset += this->path[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_forward_motion;
      u_forward_motion.base = 0;
      u_forward_motion.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->forward_motion = u_forward_motion.real;
      offset += sizeof(this->forward_motion);
      union {
        int32_t real;
        uint32_t base;
      } u_start_idx;
      u_start_idx.base = 0;
      u_start_idx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_start_idx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_start_idx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_start_idx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->start_idx = u_start_idx.real;
      offset += sizeof(this->start_idx);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->length));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->remaining_length));
      uint32_t path_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      path_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->path_length);
      if(path_lengthT > path_length)
        this->path = (geometry_msgs::PoseStamped*)realloc(this->path, path_lengthT * sizeof(geometry_msgs::PoseStamped));
      path_length = path_lengthT;
      for( uint32_t i = 0; i < path_length; i++){
      offset += this->st_path.deserialize(inbuffer + offset);
        memcpy( &(this->path[i]), &(this->st_path), sizeof(geometry_msgs::PoseStamped));
      }
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/PlanSegment"; };
    virtual const char * getMD5() override { return "2c6d67394c744f2efcb7b0a0b73ce7cc"; };

  };

}
#endif
