cmake_minimum_required(VERSION 3.5)
project(gps_waypoint_follower)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(robot_localization REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav2_util REQUIRED)

include_directories(
  include
  )

set(executable_name client_main)

add_executable(${executable_name} src/main.cpp)

set(library_name ${executable_name}_core)

#add_library(${library_name} SHARED 
#  src/gps_waypoint_follower.cpp
#)

target_link_libraries(${executable_name})

ament_target_dependencies(client_main
  rclcpp 
  example_interfaces 
  robot_localization
  geographic_msgs
  tf2_geometry_msgs
  nav2_util
)
# DIRECTORY launch params

install(
  DIRECTORY launch params 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS client_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_package()
