#ifndef _ROS_mir_actions_MirMoveBaseAction_h
#define _ROS_mir_actions_MirMoveBaseAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mir_actions/MirMoveBaseActionGoal.h"
#include "mir_actions/MirMoveBaseActionResult.h"
#include "mir_actions/MirMoveBaseActionFeedback.h"

namespace mir_actions
{

  class MirMoveBaseAction : public ros::Msg
  {
    public:
      typedef mir_actions::MirMoveBaseActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef mir_actions::MirMoveBaseActionResult _action_result_type;
      _action_result_type action_result;
      typedef mir_actions::MirMoveBaseActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    MirMoveBaseAction():
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

    virtual const char * getType() override { return "mir_actions/MirMoveBaseAction"; };
    virtual const char * getMD5() override { return "88aa1864f69098a6ee37baaf8142c203"; };

  };

}
#endif
