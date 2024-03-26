cmake_minimum_required(VERSION 3.8)
project(canopen_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Build

# Install
install(
  DIRECTORY src srv include launch urdf config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  src/test.py
  src/odom.py
  DESTINATION lib/${PROJECT_NAME})

# Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Velocity.srv")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
