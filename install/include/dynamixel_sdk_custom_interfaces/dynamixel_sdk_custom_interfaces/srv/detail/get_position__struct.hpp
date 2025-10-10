// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamixel_sdk_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_HPP_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Request __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Request __declspec(deprecated)
#endif

namespace dynamixel_sdk_custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPosition_Request_
{
  using Type = GetPosition_Request_<ContainerAllocator>;

  explicit GetPosition_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
    }
  }

  explicit GetPosition_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
    }
  }

  // field types and members
  using _id_type =
    uint8_t;
  _id_type id;

  // setters for named parameter idiom
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Request
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Request
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPosition_Request_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPosition_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPosition_Request_

// alias to use template instance with default allocator
using GetPosition_Request =
  dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dynamixel_sdk_custom_interfaces


#ifndef _WIN32
# define DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Response __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Response __declspec(deprecated)
#endif

namespace dynamixel_sdk_custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPosition_Response_
{
  using Type = GetPosition_Response_<ContainerAllocator>;

  explicit GetPosition_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position = 0l;
    }
  }

  explicit GetPosition_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position = 0l;
    }
  }

  // field types and members
  using _position_type =
    int32_t;
  _position_type position;

  // setters for named parameter idiom
  Type & set__position(
    const int32_t & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Response
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Response
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPosition_Response_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPosition_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPosition_Response_

// alias to use template instance with default allocator
using GetPosition_Response =
  dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dynamixel_sdk_custom_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Event __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Event __declspec(deprecated)
#endif

namespace dynamixel_sdk_custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPosition_Event_
{
  using Type = GetPosition_Event_<ContainerAllocator>;

  explicit GetPosition_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetPosition_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dynamixel_sdk_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Event
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_sdk_custom_interfaces__srv__GetPosition_Event
    std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPosition_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPosition_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPosition_Event_

// alias to use template instance with default allocator
using GetPosition_Event =
  dynamixel_sdk_custom_interfaces::srv::GetPosition_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dynamixel_sdk_custom_interfaces

namespace dynamixel_sdk_custom_interfaces
{

namespace srv
{

struct GetPosition
{
  using Request = dynamixel_sdk_custom_interfaces::srv::GetPosition_Request;
  using Response = dynamixel_sdk_custom_interfaces::srv::GetPosition_Response;
  using Event = dynamixel_sdk_custom_interfaces::srv::GetPosition_Event;
};

}  // namespace srv

}  // namespace dynamixel_sdk_custom_interfaces

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_HPP_
