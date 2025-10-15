#ifndef _ROS_mir_actions_MirMoveBaseActionGoal_h
#define _ROS_mir_actions_MirMoveBaseActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "mir_actions/MirMoveBaseGoal.h"

namespace mir_actions
{

  class MirMoveBaseActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef mir_actions::MirMoveBaseGoal _goal_type;
      _goal_type goal;

    MirMoveBaseActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_actions/MirMoveBaseActionGoal"; };
    virtual const char * getMD5() override { return "8ac6f5411618f50134619b87e4244699"; };

  };

}
#endif
