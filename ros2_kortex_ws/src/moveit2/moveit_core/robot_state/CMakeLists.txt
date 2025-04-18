set(MOVEIT_LIB_NAME moveit_robot_state)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/attached_body.cpp
  src/conversions.cpp
  src/robot_state.cpp
  src/cartesian_interpolator.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdf
  eigen_stl_containers
  tf2_geometry_msgs
  geometric_shapes
  urdfdom_headers
  Boost
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model
  moveit_kinematics_base
  moveit_transforms
)

install(DIRECTORY include/ DESTINATION include)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(resource_retriever REQUIRED)

  if(WIN32)
    # TODO add windows paths
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gmock(test_robot_state test/robot_state_test.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")

  target_link_libraries(test_robot_state
    moveit_test_utils
    moveit_utils
    moveit_exceptions
    ${MOVEIT_LIB_NAME}
  )

  # As an executable, this benchmark is not run as a test by default
  ament_add_gtest(test_robot_state_benchmark test/robot_state_benchmark.cpp)
  target_link_libraries(test_robot_state_benchmark
    moveit_test_utils
    moveit_utils
    moveit_exceptions
    ${MOVEIT_LIB_NAME}
  )

  ament_add_gtest(test_robot_state_complex test/test_kinematic_complex.cpp)
  target_link_libraries(test_robot_state_complex
    moveit_test_utils
    moveit_utils
    moveit_exceptions
    ${MOVEIT_LIB_NAME}
  )

  ament_add_gtest(test_aabb test/test_aabb.cpp)
  target_link_libraries(test_aabb
    moveit_test_utils
    moveit_utils
    moveit_exceptions
    ${MOVEIT_LIB_NAME}
  )

  ament_add_gtest(test_cartesian_interpolator test/test_cartesian_interpolator.cpp)
  target_link_libraries(test_cartesian_interpolator
    moveit_test_utils
    ${MOVEIT_LIB_NAME}
  )
endif()
