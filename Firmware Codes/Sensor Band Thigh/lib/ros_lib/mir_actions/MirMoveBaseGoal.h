#ifndef _ROS_mir_actions_MirMoveBaseGoal_h
#define _ROS_mir_actions_MirMoveBaseGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

namespace mir_actions
{

  class MirMoveBaseGoal : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _target_pose_type;
      _target_pose_type target_pose;
      typedef float _goal_dist_threshold_type;
      _goal_dist_threshold_type goal_dist_threshold;
      typedef float _goal_orientation_threshold_type;
      _goal_orientation_threshold_type goal_orientation_threshold;
      typedef nav_msgs::Path _path_type;
      _path_type path;
      typedef float _max_plan_time_type;
      _max_plan_time_type max_plan_time;
      typedef bool _clear_costmaps_type;
      _clear_costmaps_type clear_costmaps;

    MirMoveBaseGoal():
      target_pose(),
      goal_dist_threshold(0),
      goal_orientation_threshold(0),
      path(),
      max_plan_time(0),
      clear_costmaps(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->target_pose.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->goal_dist_threshold);
      offset += serializeAvrFloat64(outbuffer + offset, this->goal_orientation_threshold);
      offset += this->path.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_max_plan_time;
      u_max_plan_time.real = this->max_plan_time;
      *(outbuffer + offset + 0) = (u_max_plan_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_plan_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_plan_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_plan_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_plan_time);
      union {
        bool real;
        uint8_t base;
      } u_clear_costmaps;
      u_clear_costmaps.real = this->clear_costmaps;
      *(outbuffer + offset + 0) = (u_clear_costmaps.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->clear_costmaps);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->target_pose.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->goal_dist_threshold));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->goal_orientation_threshold));
      offset += this->path.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_max_plan_time;
      u_max_plan_time.base = 0;
      u_max_plan_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_plan_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_plan_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_plan_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_plan_time = u_max_plan_time.real;
      offset += sizeof(this->max_plan_time);
      union {
        bool real;
        uint8_t base;
      } u_clear_costmaps;
      u_clear_costmaps.base = 0;
      u_clear_costmaps.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->clear_costmaps = u_clear_costmaps.real;
      offset += sizeof(this->clear_costmaps);
     return offset;
    }

    virtual const char * getType() override { return "mir_actions/MirMoveBaseGoal"; };
    virtual const char * getMD5() override { return "b19f57a0f554e290b402dd0a4cdf6bf8"; };

  };

}
#endif
