#ifndef _ROS_mir_actions_MirMoveBaseResult_h
#define _ROS_mir_actions_MirMoveBaseResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace mir_actions
{

  class MirMoveBaseResult : public ros::Msg
  {
    public:
      typedef int16_t _end_state_type;
      _end_state_type end_state;
      typedef geometry_msgs::PoseStamped _end_pose_type;
      _end_pose_type end_pose;
      enum { UNDEFINED =  0 };
      enum { GOAL_REACHED =  1 };
      enum { FAILED =  -1 };
      enum { FAILED_NO_PATH =  -2 };
      enum { FAILED_GOAL_IN_STATIC_OBSTACLE =  -3 };
      enum { FAILED_GOAL_IN_FORBIDDEN_AREA =  -4 };
      enum { FAILED_GOAL_IN_DYNAMIC_OBSTACLE =  -5 };
      enum { FAILED_ROBOT_IN_COLLISION =  -6 };
      enum { FAILED_ROBOT_IN_FORBIDDEN_AREA =  -7 };
      enum { FAILED_UNKNOWN_TRAILER =  -8 };

    MirMoveBaseResult():
      end_state(0),
      end_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_end_state;
      u_end_state.real = this->end_state;
      *(outbuffer + offset + 0) = (u_end_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_end_state.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->end_state);
      offset += this->end_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_end_state;
      u_end_state.base = 0;
      u_end_state.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_end_state.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->end_state = u_end_state.real;
      offset += sizeof(this->end_state);
      offset += this->end_pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_actions/MirMoveBaseResult"; };
    virtual const char * getMD5() override { return "5d78ef95801ef85ff0171f7a59e041d7"; };

  };

}
#endif
