cmake_minimum_required(VERSION 3.8)
project(draw_letter_ur)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

set(COMMON_DEPENDENCIES
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(moveit_probe src/moveit_probe.cpp)
ament_target_dependencies(moveit_probe ${COMMON_DEPENDENCIES})

add_executable(safe_pose src/safe_pose.cpp)
ament_target_dependencies(safe_pose ${COMMON_DEPENDENCIES})

add_executable(draw_letter src/draw_letter.cpp)
ament_target_dependencies(draw_letter ${COMMON_DEPENDENCIES})

install(TARGETS moveit_probe safe_pose draw_letter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
