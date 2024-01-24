cmake_minimum_required(VERSION 3.5)
project(pubsub_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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

#Build
add_executable(pub src/publisher_member_function.cpp)
ament_target_dependencies(pub rclcpp std_msgs)

add_executable(sub src/subscriber_member_function.cpp)
ament_target_dependencies(sub rclcpp std_msgs)

#Install
install(TARGETS
  pub
  sub
  DESTINATION lib/${PROJECT_NAME})

# 폴더 경로 등록
install(DIRECTORY
  include
  src
  DESTINATION share/${PROJECT_NAME})

# cmack에 python 파일 등록, sudo chmod +x <파일명> 으로 실행 권한 줄 것
install(PROGRAMS
  src/subscriber_member_function.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
