#ifndef _ROS_SERVICE_ArrayTrajectry_h
#define _ROS_SERVICE_ArrayTrajectry_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float32MultiArray.h"
namespace array_trajectry_msgs
{
static const char ARRAYTRAJECTRY[] = "array_trajectry_msgs/ArrayTrajectry";
class ArrayTrajectryRequest : public ros::Msg
{
public:
typedef std_msgs::Float32MultiArray _array_trajectry_type;
_array_trajectry_type array_trajectry;
ArrayTrajectryRequest():
array_trajectry()
{
}
virtual int serialize(unsigned char *outbuffer) const
{
int offset = 0;
offset += this->array_trajectry.serialize(outbuffer + offset);
return offset;
}
virtual int deserialize(unsigned char *inbuffer)
{
int offset = 0;
offset += this->array_trajectry.deserialize(inbuffer + offset);
return offset;
}
const char * getType(){ return ARRAYTRAJECTRY; };
const char * getMD5(){ return "767d0ee32a8debb0fc76de7f0ea2a075"; };
};
class ArrayTrajectryResponse : public ros::Msg
{
public:
typedef bool _result_type;
_result_type result;
ArrayTrajectryResponse():
result(0)

{
}
virtual int serialize(unsigned char *outbuffer) const
{
int offset = 0;
union {
bool real;
uint8_t base;
} u_result;
u_result.real = this->result;
*(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
offset += sizeof(this->result);
return offset;
}
virtual int deserialize(unsigned char *inbuffer)
{
int offset = 0;
union {
bool real;
uint8_t base;
} u_result;
u_result.base = 0;
u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
this->result = u_result.real;
offset += sizeof(this->result);
return offset;
}
const char * getType(){ return ARRAYTRAJECTRY; };
const char * getMD5(){ return "eb13ac1f1354ccecb7941ee8fa2192e8"; };
};
class ArrayTrajectry {
public:
typedef ArrayTrajectryRequest Request;
typedef ArrayTrajectryResponse Response;
};
}
#endif