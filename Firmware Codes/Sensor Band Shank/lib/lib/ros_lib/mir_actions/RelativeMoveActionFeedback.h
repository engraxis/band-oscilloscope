#ifndef _ROS_mir_actions_RelativeMoveActionFeedback_h
#define _ROS_mir_actions_RelativeMoveActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "mir_actions/RelativeMoveFeedback.h"

namespace mir_actions
{

  class RelativeMoveActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef mir_actions::RelativeMoveFeedback _feedback_type;
      _feedback_type feedback;

    RelativeMoveActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_actions/RelativeMoveActionFeedback"; };
    virtual const char * getMD5() override { return "429713dfca2cc916179fc3088546a87b"; };

  };

}
#endif
