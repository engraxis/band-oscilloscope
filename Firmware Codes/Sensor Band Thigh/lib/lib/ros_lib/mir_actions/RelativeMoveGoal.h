#ifndef _ROS_mir_actions_RelativeMoveGoal_h
#define _ROS_mir_actions_RelativeMoveGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace mir_actions
{

  class RelativeMoveGoal : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _target_pose_type;
      _target_pose_type target_pose;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef bool _collision_detection_type;
      _collision_detection_type collision_detection;
      typedef float _disable_collision_check_dist_type;
      _disable_collision_check_dist_type disable_collision_check_dist;
      typedef float _max_linear_speed_type;
      _max_linear_speed_type max_linear_speed;
      typedef float _max_rotational_speed_type;
      _max_rotational_speed_type max_rotational_speed;
      typedef float _pid_dist_offset_type;
      _pid_dist_offset_type pid_dist_offset;
      typedef float _target_offset_type;
      _target_offset_type target_offset;
      typedef bool _only_collision_detection_type;
      _only_collision_detection_type only_collision_detection;
      typedef float _timeout_type;
      _timeout_type timeout;
      typedef bool _same_goal_type;
      _same_goal_type same_goal;

    RelativeMoveGoal():
      target_pose(),
      yaw(0),
      collision_detection(0),
      disable_collision_check_dist(0),
      max_linear_speed(0),
      max_rotational_speed(0),
      pid_dist_offset(0),
      target_offset(0),
      only_collision_detection(0),
      timeout(0),
      same_goal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->target_pose.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      union {
        bool real;
        uint8_t base;
      } u_collision_detection;
      u_collision_detection.real = this->collision_detection;
      *(outbuffer + offset + 0) = (u_collision_detection.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->collision_detection);
      offset += serializeAvrFloat64(outbuffer + offset, this->disable_collision_check_dist);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_linear_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_rotational_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->pid_dist_offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->target_offset);
      union {
        bool real;
        uint8_t base;
      } u_only_collision_detection;
      u_only_collision_detection.real = this->only_collision_detection;
      *(outbuffer + offset + 0) = (u_only_collision_detection.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->only_collision_detection);
      offset += serializeAvrFloat64(outbuffer + offset, this->timeout);
      union {
        bool real;
        uint8_t base;
      } u_same_goal;
      u_same_goal.real = this->same_goal;
      *(outbuffer + offset + 0) = (u_same_goal.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->same_goal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->target_pose.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
      union {
        bool real;
        uint8_t base;
      } u_collision_detection;
      u_collision_detection.base = 0;
      u_collision_detection.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->collision_detection = u_collision_detection.real;
      offset += sizeof(this->collision_detection);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->disable_collision_check_dist));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_linear_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_rotational_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pid_dist_offset));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_offset));
      union {
        bool real;
        uint8_t base;
      } u_only_collision_detection;
      u_only_collision_detection.base = 0;
      u_only_collision_detection.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->only_collision_detection = u_only_collision_detection.real;
      offset += sizeof(this->only_collision_detection);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->timeout));
      union {
        bool real;
        uint8_t base;
      } u_same_goal;
      u_same_goal.base = 0;
      u_same_goal.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->same_goal = u_same_goal.real;
      offset += sizeof(this->same_goal);
     return offset;
    }

    virtual const char * getType() override { return "mir_actions/RelativeMoveGoal"; };
    virtual const char * getMD5() override { return "ceef9ff5cf266b0a2f52eea13fa5ba8a"; };

  };

}
#endif
