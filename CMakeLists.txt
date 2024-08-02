cmake_minimum_required(VERSION 3.8)
project(route_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_library(
  route_generator SHARED
  src/route.cpp
)

ament_target_dependencies(
  route_generator
  rclcpp
  rclcpp_components
  std_msgs
  autoware_planning_msgs
  nav_msgs
)

rclcpp_components_register_nodes(
  route_generator
  "route_component::RoutePublisher"
)

install(TARGETS 
  route_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
