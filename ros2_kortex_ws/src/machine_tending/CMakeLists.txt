cmake_minimum_required(VERSION 3.8)
project(machine_tending)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ Dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

# Python Dependencies
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# C++ Executable
add_executable(machine_tending_node src/machine_tending_node.cpp)
target_include_directories(machine_tending_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(machine_tending_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  machine_tending_node
  "moveit_task_constructor_core"
  "rclcpp"
)

install(TARGETS machine_tending_node
  DESTINATION lib/${PROJECT_NAME})

# Python install
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  machine_tending/relay_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  machine_tending/arduino_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
