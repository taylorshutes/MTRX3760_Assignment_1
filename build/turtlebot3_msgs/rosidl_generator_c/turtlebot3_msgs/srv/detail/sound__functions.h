// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from turtlebot3_msgs:srv/Sound.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "turtlebot3_msgs/srv/sound.h"


#ifndef TURTLEBOT3_MSGS__SRV__DETAIL__SOUND__FUNCTIONS_H_
#define TURTLEBOT3_MSGS__SRV__DETAIL__SOUND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "turtlebot3_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "turtlebot3_msgs/srv/detail/sound__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__srv__Sound__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__srv__Sound__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__srv__Sound__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__srv__Sound__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize srv/Sound message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * turtlebot3_msgs__srv__Sound_Request
 * )) before or use
 * turtlebot3_msgs__srv__Sound_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Request__init(turtlebot3_msgs__srv__Sound_Request * msg);

/// Finalize srv/Sound message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Request__fini(turtlebot3_msgs__srv__Sound_Request * msg);

/// Create srv/Sound message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * turtlebot3_msgs__srv__Sound_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__srv__Sound_Request *
turtlebot3_msgs__srv__Sound_Request__create(void);

/// Destroy srv/Sound message.
/**
 * It calls
 * turtlebot3_msgs__srv__Sound_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Request__destroy(turtlebot3_msgs__srv__Sound_Request * msg);

/// Check for srv/Sound message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Request__are_equal(const turtlebot3_msgs__srv__Sound_Request * lhs, const turtlebot3_msgs__srv__Sound_Request * rhs);

/// Copy a srv/Sound message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Request__copy(
  const turtlebot3_msgs__srv__Sound_Request * input,
  turtlebot3_msgs__srv__Sound_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__srv__Sound_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__srv__Sound_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__srv__Sound_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__srv__Sound_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/Sound messages.
/**
 * It allocates the memory for the number of elements and calls
 * turtlebot3_msgs__srv__Sound_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Request__Sequence__init(turtlebot3_msgs__srv__Sound_Request__Sequence * array, size_t size);

/// Finalize array of srv/Sound messages.
/**
 * It calls
 * turtlebot3_msgs__srv__Sound_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Request__Sequence__fini(turtlebot3_msgs__srv__Sound_Request__Sequence * array);

/// Create array of srv/Sound messages.
/**
 * It allocates the memory for the array and calls
 * turtlebot3_msgs__srv__Sound_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__srv__Sound_Request__Sequence *
turtlebot3_msgs__srv__Sound_Request__Sequence__create(size_t size);

/// Destroy array of srv/Sound messages.
/**
 * It calls
 * turtlebot3_msgs__srv__Sound_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Request__Sequence__destroy(turtlebot3_msgs__srv__Sound_Request__Sequence * array);

/// Check for srv/Sound message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Request__Sequence__are_equal(const turtlebot3_msgs__srv__Sound_Request__Sequence * lhs, const turtlebot3_msgs__srv__Sound_Request__Sequence * rhs);

/// Copy an array of srv/Sound messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Request__Sequence__copy(
  const turtlebot3_msgs__srv__Sound_Request__Sequence * input,
  turtlebot3_msgs__srv__Sound_Request__Sequence * output);

/// Initialize srv/Sound message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * turtlebot3_msgs__srv__Sound_Response
 * )) before or use
 * turtlebot3_msgs__srv__Sound_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Response__init(turtlebot3_msgs__srv__Sound_Response * msg);

/// Finalize srv/Sound message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Response__fini(turtlebot3_msgs__srv__Sound_Response * msg);

/// Create srv/Sound message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * turtlebot3_msgs__srv__Sound_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__srv__Sound_Response *
turtlebot3_msgs__srv__Sound_Response__create(void);

/// Destroy srv/Sound message.
/**
 * It calls
 * turtlebot3_msgs__srv__Sound_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Response__destroy(turtlebot3_msgs__srv__Sound_Response * msg);

/// Check for srv/Sound message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Response__are_equal(const turtlebot3_msgs__srv__Sound_Response * lhs, const turtlebot3_msgs__srv__Sound_Response * rhs);

/// Copy a srv/Sound message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Response__copy(
  const turtlebot3_msgs__srv__Sound_Response * input,
  turtlebot3_msgs__srv__Sound_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__srv__Sound_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__srv__Sound_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__srv__Sound_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__srv__Sound_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/Sound messages.
/**
 * It allocates the memory for the number of elements and calls
 * turtlebot3_msgs__srv__Sound_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Response__Sequence__init(turtlebot3_msgs__srv__Sound_Response__Sequence * array, size_t size);

/// Finalize array of srv/Sound messages.
/**
 * It calls
 * turtlebot3_msgs__srv__Sound_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Response__Sequence__fini(turtlebot3_msgs__srv__Sound_Response__Sequence * array);

/// Create array of srv/Sound messages.
/**
 * It allocates the memory for the array and calls
 * turtlebot3_msgs__srv__Sound_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__srv__Sound_Response__Sequence *
turtlebot3_msgs__srv__Sound_Response__Sequence__create(size_t size);

/// Destroy array of srv/Sound messages.
/**
 * It calls
 * turtlebot3_msgs__srv__Sound_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Response__Sequence__destroy(turtlebot3_msgs__srv__Sound_Response__Sequence * array);

/// Check for srv/Sound message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Response__Sequence__are_equal(const turtlebot3_msgs__srv__Sound_Response__Sequence * lhs, const turtlebot3_msgs__srv__Sound_Response__Sequence * rhs);

/// Copy an array of srv/Sound messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Response__Sequence__copy(
  const turtlebot3_msgs__srv__Sound_Response__Sequence * input,
  turtlebot3_msgs__srv__Sound_Response__Sequence * output);

/// Initialize srv/Sound message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * turtlebot3_msgs__srv__Sound_Event
 * )) before or use
 * turtlebot3_msgs__srv__Sound_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Event__init(turtlebot3_msgs__srv__Sound_Event * msg);

/// Finalize srv/Sound message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Event__fini(turtlebot3_msgs__srv__Sound_Event * msg);

/// Create srv/Sound message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * turtlebot3_msgs__srv__Sound_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__srv__Sound_Event *
turtlebot3_msgs__srv__Sound_Event__create(void);

/// Destroy srv/Sound message.
/**
 * It calls
 * turtlebot3_msgs__srv__Sound_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Event__destroy(turtlebot3_msgs__srv__Sound_Event * msg);

/// Check for srv/Sound message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Event__are_equal(const turtlebot3_msgs__srv__Sound_Event * lhs, const turtlebot3_msgs__srv__Sound_Event * rhs);

/// Copy a srv/Sound message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Event__copy(
  const turtlebot3_msgs__srv__Sound_Event * input,
  turtlebot3_msgs__srv__Sound_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_type_hash_t *
turtlebot3_msgs__srv__Sound_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeDescription *
turtlebot3_msgs__srv__Sound_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeSource *
turtlebot3_msgs__srv__Sound_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
turtlebot3_msgs__srv__Sound_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/Sound messages.
/**
 * It allocates the memory for the number of elements and calls
 * turtlebot3_msgs__srv__Sound_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Event__Sequence__init(turtlebot3_msgs__srv__Sound_Event__Sequence * array, size_t size);

/// Finalize array of srv/Sound messages.
/**
 * It calls
 * turtlebot3_msgs__srv__Sound_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Event__Sequence__fini(turtlebot3_msgs__srv__Sound_Event__Sequence * array);

/// Create array of srv/Sound messages.
/**
 * It allocates the memory for the array and calls
 * turtlebot3_msgs__srv__Sound_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
turtlebot3_msgs__srv__Sound_Event__Sequence *
turtlebot3_msgs__srv__Sound_Event__Sequence__create(size_t size);

/// Destroy array of srv/Sound messages.
/**
 * It calls
 * turtlebot3_msgs__srv__Sound_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
void
turtlebot3_msgs__srv__Sound_Event__Sequence__destroy(turtlebot3_msgs__srv__Sound_Event__Sequence * array);

/// Check for srv/Sound message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Event__Sequence__are_equal(const turtlebot3_msgs__srv__Sound_Event__Sequence * lhs, const turtlebot3_msgs__srv__Sound_Event__Sequence * rhs);

/// Copy an array of srv/Sound messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtlebot3_msgs
bool
turtlebot3_msgs__srv__Sound_Event__Sequence__copy(
  const turtlebot3_msgs__srv__Sound_Event__Sequence * input,
  turtlebot3_msgs__srv__Sound_Event__Sequence * output);
#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_MSGS__SRV__DETAIL__SOUND__FUNCTIONS_H_
