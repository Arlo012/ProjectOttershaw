#ifndef _ROS_ottershaw_masta_Servo_h
#define _ROS_ottershaw_masta_Servo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ottershaw_masta
{

  class Servo : public ros::Msg
  {
    public:
      int16_t ID;
      int16_t angle;
      int16_t stepSize;

    Servo():
      ID(0),
      angle(0),
      stepSize(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_ID;
      u_ID.real = this->ID;
      *(outbuffer + offset + 0) = (u_ID.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ID.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ID);
      union {
        int16_t real;
        uint16_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angle);
      union {
        int16_t real;
        uint16_t base;
      } u_stepSize;
      u_stepSize.real = this->stepSize;
      *(outbuffer + offset + 0) = (u_stepSize.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stepSize.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->stepSize);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_ID;
      u_ID.base = 0;
      u_ID.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ID.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ID = u_ID.real;
      offset += sizeof(this->ID);
      union {
        int16_t real;
        uint16_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      union {
        int16_t real;
        uint16_t base;
      } u_stepSize;
      u_stepSize.base = 0;
      u_stepSize.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stepSize.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stepSize = u_stepSize.real;
      offset += sizeof(this->stepSize);
     return offset;
    }

    const char * getType(){ return "ottershaw_masta/Servo"; };
    const char * getMD5(){ return "067a4b40f3100f89a5c296ae53f7c9a4"; };

  };

}
#endif