#ifndef _ROS_geometry_msgs_TwistWithCovariance_h
#define _ROS_geometry_msgs_TwistWithCovariance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Twist.h"

namespace geometry_msgs
{

  class TwistWithCovariance : public ros::Msg
  {
    public:
      geometry_msgs::Twist twist;
      float covariance[36];

    TwistWithCovariance():
      twist(),
