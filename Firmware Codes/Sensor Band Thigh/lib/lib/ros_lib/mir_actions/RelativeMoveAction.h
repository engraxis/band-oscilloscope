#ifndef _ROS_mir_actions_RelativeMoveAction_h
#define _ROS_mir_actions_RelativeMoveAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mir_actions/RelativeMoveActionGoal.h"
#include "mir_actions/RelativeMoveActionResult.h"
#include "mir_actions/RelativeMoveActionFeedback.h"

namespace mir_actions
{

  class RelativeMoveAction : public ros::Msg
  {
    public:
      typedef mir_actions::RelativeMoveActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef mir_actions::RelativeMoveActionResult _action_result_type;
      _action_result_type action_result;
      typedef mir_actions::RelativeMoveActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    RelativeMoveAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_actions/RelativeMoveAction"; };
    virtual const char * getMD5() override { return "d640e6845b2a68f81ebb9c385eefcb7e"; };

  };

}
#endif
