#ifndef _ROS_mir_msgs_PlanSegments_h
#define _ROS_mir_msgs_PlanSegments_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mir_msgs/PlanSegment.h"

namespace mir_msgs
{

  class PlanSegments : public ros::Msg
  {
    public:
      uint32_t p_segments_length;
      typedef mir_msgs::PlanSegment _p_segments_type;
      _p_segments_type st_p_segments;
      _p_segments_type * p_segments;

    PlanSegments():
      p_segments_length(0), st_p_segments(), p_segments(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->p_segments_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->p_segments_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->p_segments_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->p_segments_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_segments_length);
      for( uint32_t i = 0; i < p_segments_length; i++){
      offset += this->p_segments[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t p_segments_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      p_segments_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      p_segments_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      p_segments_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->p_segments_length);
      if(p_segments_lengthT > p_segments_length)
        this->p_segments = (mir_msgs::PlanSegment*)realloc(this->p_segments, p_segments_lengthT * sizeof(mir_msgs::PlanSegment));
      p_segments_length = p_segments_lengthT;
      for( uint32_t i = 0; i < p_segments_length; i++){
      offset += this->st_p_segments.deserialize(inbuffer + offset);
        memcpy( &(this->p_segments[i]), &(this->st_p_segments), sizeof(mir_msgs::PlanSegment));
      }
     return offset;
    }

    virtual const char * getType() override { return "mir_msgs/PlanSegments"; };
    virtual const char * getMD5() override { return "9176305685849eadfd34548fd6b41d90"; };

  };

}
#endif
