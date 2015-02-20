#ifndef _ROS_ottershaw_masta_Analog_h
#define _ROS_ottershaw_masta_Analog_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ottershaw_masta
{

  class Analog : public ros::Msg
  {
    public:
      int16_t id;
      int16_t value;

    Analog():
      id(0),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int16_t real;
        uint16_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        int16_t real;
        uint16_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->value = u_value.real;
      offset += sizeof(this->value);
     return offset;
    }

    const char * getType(){ return "ottershaw_masta/Analog"; };
    const char * getMD5(){ return "ccbf596d1886375c8a6adf47512fb4c4"; };

  };

}
#endif