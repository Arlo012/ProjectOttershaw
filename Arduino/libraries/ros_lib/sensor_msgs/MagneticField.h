#ifndef _ROS_sensor_msgs_MagneticField_h
#define _ROS_sensor_msgs_MagneticField_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace sensor_msgs
{

  class MagneticField : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Vector3 magnetic_field;
      float magnetic_field_covariance[9];

    MagneticField():
      header(),
      magnetic_field(),
