#ifndef _ROS_mir_actions_MirMoveBaseActionResult_h
#define _ROS_mir_actions_MirMoveBaseActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "mir_actions/MirMoveBaseResult.h"

namespace mir_actions
{

  class MirMoveBaseActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef mir_actions::MirMoveBaseResult _result_type;
      _result_type result;

    MirMoveBaseActionResult():
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

    virtual const char * getType() override { return "mir_actions/MirMoveBaseActionResult"; };
    virtual const char * getMD5() override { return "bcf56cc2bd559c55ce07eb68272d1009"; };

  };

}
#endif
