cmake_minimum_required(VERSION 3.8)
project(cpp_humble)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build
add_executable(publisher_member_function src/publisher_member_function.cpp)
ament_target_dependencies(publisher_member_function rclcpp std_msgs)
add_executable(subscriber_member_function src/subscriber_member_function.cpp)
ament_target_dependencies(subscriber_member_function rclcpp std_msgs)

# Install
install(TARGETS
  publisher_member_function
  subscriber_member_function
  DESTINATION lib/${PROJECT_NAME})

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
