cmake_minimum_required(VERSION 3.5)
project(gps_waypoint_follower)

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

find_package(ament_cmake REQUIRED)
find_package(robot_localization REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gps_interfaces REQUIRED)

nav2_package()

link_libraries(stdc++fs)

include_directories(
  include
)

set(executable_name gps_waypoint_follower)

add_executable(${executable_name} src/main.cpp)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED 
  src/gps_waypoint_follower.cpp
)

set(dependencies 
  rclcpp 
  rclcpp_action
  rclcpp_lifecycle
  nav2_util
  nav2_lifecycle_manager
  nav_msgs
  geographic_msgs
  nav2_msgs
  nav2_core      
  nav2_waypoint_follower
  tf2_ros
  tf2_geometry_msgs
  robot_localization
  gps_interfaces
)


ament_target_dependencies(${executable_name} 
  ${dependencies} 
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name} ${dependencies})

rclcpp_components_register_nodes(${library_name} 
  "gps_waypoint_follower::GPSWaypointFollower")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
