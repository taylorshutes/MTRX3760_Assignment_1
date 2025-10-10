// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_sdk_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__BUILDER_HPP_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_sdk_custom_interfaces/srv/detail/get_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_sdk_custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetPosition_Request_id
{
public:
  Init_GetPosition_Request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dynamixel_sdk_custom_interfaces::srv::GetPosition_Request id(::dynamixel_sdk_custom_interfaces::srv::GetPosition_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::srv::GetPosition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_sdk_custom_interfaces::srv::GetPosition_Request>()
{
  return dynamixel_sdk_custom_interfaces::srv::builder::Init_GetPosition_Request_id();
}

}  // namespace dynamixel_sdk_custom_interfaces


namespace dynamixel_sdk_custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetPosition_Response_position
{
public:
  Init_GetPosition_Response_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dynamixel_sdk_custom_interfaces::srv::GetPosition_Response position(::dynamixel_sdk_custom_interfaces::srv::GetPosition_Response::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::srv::GetPosition_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_sdk_custom_interfaces::srv::GetPosition_Response>()
{
  return dynamixel_sdk_custom_interfaces::srv::builder::Init_GetPosition_Response_position();
}

}  // namespace dynamixel_sdk_custom_interfaces


namespace dynamixel_sdk_custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetPosition_Event_response
{
public:
  explicit Init_GetPosition_Event_response(::dynamixel_sdk_custom_interfaces::srv::GetPosition_Event & msg)
  : msg_(msg)
  {}
  ::dynamixel_sdk_custom_interfaces::srv::GetPosition_Event response(::dynamixel_sdk_custom_interfaces::srv::GetPosition_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::srv::GetPosition_Event msg_;
};

class Init_GetPosition_Event_request
{
public:
  explicit Init_GetPosition_Event_request(::dynamixel_sdk_custom_interfaces::srv::GetPosition_Event & msg)
  : msg_(msg)
  {}
  Init_GetPosition_Event_response request(::dynamixel_sdk_custom_interfaces::srv::GetPosition_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetPosition_Event_response(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::srv::GetPosition_Event msg_;
};

class Init_GetPosition_Event_info
{
public:
  Init_GetPosition_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPosition_Event_request info(::dynamixel_sdk_custom_interfaces::srv::GetPosition_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetPosition_Event_request(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::srv::GetPosition_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_sdk_custom_interfaces::srv::GetPosition_Event>()
{
  return dynamixel_sdk_custom_interfaces::srv::builder::Init_GetPosition_Event_info();
}

}  // namespace dynamixel_sdk_custom_interfaces

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__BUILDER_HPP_
