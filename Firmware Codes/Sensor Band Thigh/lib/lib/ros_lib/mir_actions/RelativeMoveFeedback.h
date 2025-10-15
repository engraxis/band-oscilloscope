#ifndef _ROS_mir_actions_RelativeMoveFeedback_h
#define _ROS_mir_actions_RelativeMoveFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace mir_actions
{

  class RelativeMoveFeedback : public ros::Msg
  {
    public:
      typedef int8_t _state_type;
      _state_type state;
      typedef geometry_msgs::PoseStamped _current_goal_type;
      _current_goal_type current_goal;
      typedef geometry_msgs::PoseStamped _dist_to_goal_type;
      _dist_to_goal_type dist_to_goal;
      enum { DOCKING =  0 };
      enum { COLLISION =  1 };

    RelativeMoveFeedback():
      state(0),
      current_goal(),
      dist_to_goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      offset += this->current_goal.serialize(outbuffer + offset);
      offset += this->dist_to_goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
      offset += this->current_goal.deserialize(inbuffer + offset);
      offset += this->dist_to_goal.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_actions/RelativeMoveFeedback"; };
    virtual const char * getMD5() override { return "95b697bacf828ff88c46362efe2f6b7e"; };

  };

}
#endif
