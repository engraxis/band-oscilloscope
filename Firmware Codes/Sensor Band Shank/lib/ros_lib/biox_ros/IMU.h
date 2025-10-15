#ifndef _ROS_biox_ros_IMU_h
#define _ROS_biox_ros_IMU_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace biox_ros
{

  class IMU : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _gravity_type;
      _gravity_type gravity;
      typedef geometry_msgs::Point _gyro_type;
      _gyro_type gyro;
      typedef geometry_msgs::Point _linear_acceleration_type;
      _linear_acceleration_type linear_acceleration;
      typedef geometry_msgs::Point _euler_type;
      _euler_type euler;

    IMU():
      gravity(),
      gyro(),
      linear_acceleration(),
      euler()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->gravity.serialize(outbuffer + offset);
      offset += this->gyro.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      offset += this->euler.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->gravity.deserialize(inbuffer + offset);
      offset += this->gyro.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      offset += this->euler.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "biox_ros/IMU"; };
    virtual const char * getMD5() override { return "d35449a99e8eb176f8079e5e1601693d"; };

  };

}
#endif
