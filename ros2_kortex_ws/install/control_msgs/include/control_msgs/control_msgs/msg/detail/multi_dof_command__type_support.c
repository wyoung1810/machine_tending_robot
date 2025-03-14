// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from control_msgs:msg/MultiDOFCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "control_msgs/msg/detail/multi_dof_command__rosidl_typesupport_introspection_c.h"
#include "control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "control_msgs/msg/detail/multi_dof_command__functions.h"
#include "control_msgs/msg/detail/multi_dof_command__struct.h"


// Include directives for member types
// Member `dof_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `values`
// Member `values_dot`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  control_msgs__msg__MultiDOFCommand__init(message_memory);
}

void control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_fini_function(void * message_memory)
{
  control_msgs__msg__MultiDOFCommand__fini(message_memory);
}

size_t control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__size_function__MultiDOFCommand__dof_names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_const_function__MultiDOFCommand__dof_names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_function__MultiDOFCommand__dof_names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__fetch_function__MultiDOFCommand__dof_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_const_function__MultiDOFCommand__dof_names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__assign_function__MultiDOFCommand__dof_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_function__MultiDOFCommand__dof_names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__resize_function__MultiDOFCommand__dof_names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__size_function__MultiDOFCommand__values(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_const_function__MultiDOFCommand__values(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_function__MultiDOFCommand__values(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__fetch_function__MultiDOFCommand__values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_const_function__MultiDOFCommand__values(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__assign_function__MultiDOFCommand__values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_function__MultiDOFCommand__values(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__resize_function__MultiDOFCommand__values(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__size_function__MultiDOFCommand__values_dot(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_const_function__MultiDOFCommand__values_dot(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_function__MultiDOFCommand__values_dot(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__fetch_function__MultiDOFCommand__values_dot(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_const_function__MultiDOFCommand__values_dot(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__assign_function__MultiDOFCommand__values_dot(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_function__MultiDOFCommand__values_dot(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__resize_function__MultiDOFCommand__values_dot(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_message_member_array[3] = {
  {
    "dof_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs__msg__MultiDOFCommand, dof_names),  // bytes offset in struct
    NULL,  // default value
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__size_function__MultiDOFCommand__dof_names,  // size() function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_const_function__MultiDOFCommand__dof_names,  // get_const(index) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_function__MultiDOFCommand__dof_names,  // get(index) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__fetch_function__MultiDOFCommand__dof_names,  // fetch(index, &value) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__assign_function__MultiDOFCommand__dof_names,  // assign(index, value) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__resize_function__MultiDOFCommand__dof_names  // resize(index) function pointer
  },
  {
    "values",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs__msg__MultiDOFCommand, values),  // bytes offset in struct
    NULL,  // default value
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__size_function__MultiDOFCommand__values,  // size() function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_const_function__MultiDOFCommand__values,  // get_const(index) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_function__MultiDOFCommand__values,  // get(index) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__fetch_function__MultiDOFCommand__values,  // fetch(index, &value) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__assign_function__MultiDOFCommand__values,  // assign(index, value) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__resize_function__MultiDOFCommand__values  // resize(index) function pointer
  },
  {
    "values_dot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs__msg__MultiDOFCommand, values_dot),  // bytes offset in struct
    NULL,  // default value
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__size_function__MultiDOFCommand__values_dot,  // size() function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_const_function__MultiDOFCommand__values_dot,  // get_const(index) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__get_function__MultiDOFCommand__values_dot,  // get(index) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__fetch_function__MultiDOFCommand__values_dot,  // fetch(index, &value) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__assign_function__MultiDOFCommand__values_dot,  // assign(index, value) function pointer
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__resize_function__MultiDOFCommand__values_dot  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_message_members = {
  "control_msgs__msg",  // message namespace
  "MultiDOFCommand",  // message name
  3,  // number of fields
  sizeof(control_msgs__msg__MultiDOFCommand),
  control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_message_member_array,  // message members
  control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_message_type_support_handle = {
  0,
  &control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, control_msgs, msg, MultiDOFCommand)() {
  if (!control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_message_type_support_handle.typesupport_identifier) {
    control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &control_msgs__msg__MultiDOFCommand__rosidl_typesupport_introspection_c__MultiDOFCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
