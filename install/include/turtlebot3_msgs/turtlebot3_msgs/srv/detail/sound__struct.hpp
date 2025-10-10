// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_msgs:srv/Sound.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "turtlebot3_msgs/srv/sound.hpp"


#ifndef TURTLEBOT3_MSGS__SRV__DETAIL__SOUND__STRUCT_HPP_
#define TURTLEBOT3_MSGS__SRV__DETAIL__SOUND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__srv__Sound_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__srv__Sound_Request __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Sound_Request_
{
  using Type = Sound_Request_<ContainerAllocator>;

  explicit Sound_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->value = 0;
    }
  }

  explicit Sound_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->value = 0;
    }
  }

  // field types and members
  using _value_type =
    uint8_t;
  _value_type value;

  // setters for named parameter idiom
  Type & set__value(
    const uint8_t & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__srv__Sound_Request
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__srv__Sound_Request
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sound_Request_ & other) const
  {
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sound_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sound_Request_

// alias to use template instance with default allocator
using Sound_Request =
  turtlebot3_msgs::srv::Sound_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_msgs


#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__srv__Sound_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__srv__Sound_Response __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Sound_Response_
{
  using Type = Sound_Response_<ContainerAllocator>;

  explicit Sound_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit Sound_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__srv__Sound_Response
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__srv__Sound_Response
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sound_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sound_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sound_Response_

// alias to use template instance with default allocator
using Sound_Response =
  turtlebot3_msgs::srv::Sound_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__srv__Sound_Event __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__srv__Sound_Event __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Sound_Event_
{
  using Type = Sound_Event_<ContainerAllocator>;

  explicit Sound_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Sound_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<turtlebot3_msgs::srv::Sound_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<turtlebot3_msgs::srv::Sound_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__srv__Sound_Event
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__srv__Sound_Event
    std::shared_ptr<turtlebot3_msgs::srv::Sound_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sound_Event_ & other) const
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
  bool operator!=(const Sound_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sound_Event_

// alias to use template instance with default allocator
using Sound_Event =
  turtlebot3_msgs::srv::Sound_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_msgs

namespace turtlebot3_msgs
{

namespace srv
{

struct Sound
{
  using Request = turtlebot3_msgs::srv::Sound_Request;
  using Response = turtlebot3_msgs::srv::Sound_Response;
  using Event = turtlebot3_msgs::srv::Sound_Event;
};

}  // namespace srv

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__SRV__DETAIL__SOUND__STRUCT_HPP_
