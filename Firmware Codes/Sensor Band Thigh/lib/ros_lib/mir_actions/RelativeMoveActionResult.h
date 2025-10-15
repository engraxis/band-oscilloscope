#ifndef _ROS_mir_actions_RelativeMoveActionResult_h
#define _ROS_mir_actions_RelativeMoveActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "mir_actions/RelativeMoveResult.h"

namespace mir_actions
{

  class RelativeMoveActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef mir_actions::RelativeMoveResult _result_type;
      _result_type result;

    RelativeMoveActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mir_actions/RelativeMoveActionResult"; };
    virtual const char * getMD5() override { return "5831b2188de0c22dbb0a798f966d28db"; };

  };

}
#endif
