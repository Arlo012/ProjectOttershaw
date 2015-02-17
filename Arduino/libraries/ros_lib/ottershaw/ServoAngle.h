#ifndef _ROS_ottershaw_ServoAngle_h
#define _ROS_ottershaw_ServoAngle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ottershaw
{

  class ServoAngle : public ros::Msg
  {
    public:
      uint8_t id;
      uint8_t position;
      uint8_t speed;

    ServoAngle():
      id(0),
      position(0),
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->position >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position);
      *(outbuffer + offset + 0) = (this->speed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      this->position =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->position);
      this->speed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->speed);
     return offset;
    }

    const char * getType(){ return "ottershaw/ServoAngle"; };
    const char * getMD5(){ return "f4a7adf216595ca2affe010927e05b8d"; };

  };

}
#endif