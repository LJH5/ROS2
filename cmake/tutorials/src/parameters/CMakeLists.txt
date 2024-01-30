cmake_minimum_required(VERSION 3.5)
project(parameters)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Build
add_executable(minimal_param_node src/parameters_node.cpp)
ament_target_dependencies(minimal_param_node rclcpp)

# Install
install(TARGETS
  minimal_param_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch param
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
