// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mbam_interfaces:msg/TargetBeliefs.idl
// generated code does not contain a copyright notice

#ifndef MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__FUNCTIONS_H_
#define MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mbam_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "mbam_interfaces/msg/detail/target_beliefs__struct.h"

/// Initialize msg/TargetBeliefs message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mbam_interfaces__msg__TargetBeliefs
 * )) before or use
 * mbam_interfaces__msg__TargetBeliefs__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
bool
mbam_interfaces__msg__TargetBeliefs__init(mbam_interfaces__msg__TargetBeliefs * msg);

/// Finalize msg/TargetBeliefs message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
void
mbam_interfaces__msg__TargetBeliefs__fini(mbam_interfaces__msg__TargetBeliefs * msg);

/// Create msg/TargetBeliefs message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mbam_interfaces__msg__TargetBeliefs__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
mbam_interfaces__msg__TargetBeliefs *
mbam_interfaces__msg__TargetBeliefs__create();

/// Destroy msg/TargetBeliefs message.
/**
 * It calls
 * mbam_interfaces__msg__TargetBeliefs__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
void
mbam_interfaces__msg__TargetBeliefs__destroy(mbam_interfaces__msg__TargetBeliefs * msg);

/// Check for msg/TargetBeliefs message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
bool
mbam_interfaces__msg__TargetBeliefs__are_equal(const mbam_interfaces__msg__TargetBeliefs * lhs, const mbam_interfaces__msg__TargetBeliefs * rhs);

/// Copy a msg/TargetBeliefs message.
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
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
bool
mbam_interfaces__msg__TargetBeliefs__copy(
  const mbam_interfaces__msg__TargetBeliefs * input,
  mbam_interfaces__msg__TargetBeliefs * output);

/// Initialize array of msg/TargetBeliefs messages.
/**
 * It allocates the memory for the number of elements and calls
 * mbam_interfaces__msg__TargetBeliefs__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
bool
mbam_interfaces__msg__TargetBeliefs__Sequence__init(mbam_interfaces__msg__TargetBeliefs__Sequence * array, size_t size);

/// Finalize array of msg/TargetBeliefs messages.
/**
 * It calls
 * mbam_interfaces__msg__TargetBeliefs__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
void
mbam_interfaces__msg__TargetBeliefs__Sequence__fini(mbam_interfaces__msg__TargetBeliefs__Sequence * array);

/// Create array of msg/TargetBeliefs messages.
/**
 * It allocates the memory for the array and calls
 * mbam_interfaces__msg__TargetBeliefs__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
mbam_interfaces__msg__TargetBeliefs__Sequence *
mbam_interfaces__msg__TargetBeliefs__Sequence__create(size_t size);

/// Destroy array of msg/TargetBeliefs messages.
/**
 * It calls
 * mbam_interfaces__msg__TargetBeliefs__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
void
mbam_interfaces__msg__TargetBeliefs__Sequence__destroy(mbam_interfaces__msg__TargetBeliefs__Sequence * array);

/// Check for msg/TargetBeliefs message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
bool
mbam_interfaces__msg__TargetBeliefs__Sequence__are_equal(const mbam_interfaces__msg__TargetBeliefs__Sequence * lhs, const mbam_interfaces__msg__TargetBeliefs__Sequence * rhs);

/// Copy an array of msg/TargetBeliefs messages.
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
ROSIDL_GENERATOR_C_PUBLIC_mbam_interfaces
bool
mbam_interfaces__msg__TargetBeliefs__Sequence__copy(
  const mbam_interfaces__msg__TargetBeliefs__Sequence * input,
  mbam_interfaces__msg__TargetBeliefs__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MBAM_INTERFACES__MSG__DETAIL__TARGET_BELIEFS__FUNCTIONS_H_
