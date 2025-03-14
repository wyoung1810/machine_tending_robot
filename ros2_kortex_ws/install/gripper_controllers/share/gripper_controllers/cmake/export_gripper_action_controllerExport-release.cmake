#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gripper_controllers::gripper_action_controller" for configuration "Release"
set_property(TARGET gripper_controllers::gripper_action_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(gripper_controllers::gripper_action_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libgripper_action_controller.so"
  IMPORTED_SONAME_RELEASE "libgripper_action_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS gripper_controllers::gripper_action_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_gripper_controllers::gripper_action_controller "${_IMPORT_PREFIX}/lib/libgripper_action_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
