// Generated by gencpp from file snake_control/PublishJointCmds.msg
// DO NOT EDIT!


#ifndef SNAKE_CONTROL_MESSAGE_PUBLISHJOINTCMDS_H
#define SNAKE_CONTROL_MESSAGE_PUBLISHJOINTCMDS_H

#include <ros/service_traits.h>


#include <snake_control/PublishJointCmdsRequest.h>
#include <snake_control/PublishJointCmdsResponse.h>


namespace snake_control
{

struct PublishJointCmds
{

typedef PublishJointCmdsRequest Request;
typedef PublishJointCmdsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct PublishJointCmds
} // namespace snake_control


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::snake_control::PublishJointCmds > {
  static const char* value()
  {
    return "a95cf7713f743cc252b964b3db9e31ef";
  }

  static const char* value(const ::snake_control::PublishJointCmds&) { return value(); }
};

template<>
struct DataType< ::snake_control::PublishJointCmds > {
  static const char* value()
  {
    return "snake_control/PublishJointCmds";
  }

  static const char* value(const ::snake_control::PublishJointCmds&) { return value(); }
};


// service_traits::MD5Sum< ::snake_control::PublishJointCmdsRequest> should match
// service_traits::MD5Sum< ::snake_control::PublishJointCmds >
template<>
struct MD5Sum< ::snake_control::PublishJointCmdsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::snake_control::PublishJointCmds >::value();
  }
  static const char* value(const ::snake_control::PublishJointCmdsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::snake_control::PublishJointCmdsRequest> should match
// service_traits::DataType< ::snake_control::PublishJointCmds >
template<>
struct DataType< ::snake_control::PublishJointCmdsRequest>
{
  static const char* value()
  {
    return DataType< ::snake_control::PublishJointCmds >::value();
  }
  static const char* value(const ::snake_control::PublishJointCmdsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::snake_control::PublishJointCmdsResponse> should match
// service_traits::MD5Sum< ::snake_control::PublishJointCmds >
template<>
struct MD5Sum< ::snake_control::PublishJointCmdsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::snake_control::PublishJointCmds >::value();
  }
  static const char* value(const ::snake_control::PublishJointCmdsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::snake_control::PublishJointCmdsResponse> should match
// service_traits::DataType< ::snake_control::PublishJointCmds >
template<>
struct DataType< ::snake_control::PublishJointCmdsResponse>
{
  static const char* value()
  {
    return DataType< ::snake_control::PublishJointCmds >::value();
  }
  static const char* value(const ::snake_control::PublishJointCmdsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SNAKE_CONTROL_MESSAGE_PUBLISHJOINTCMDS_H
