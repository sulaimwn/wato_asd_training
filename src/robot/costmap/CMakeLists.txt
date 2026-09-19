cmake_minimum_required(VERSION 3.10)
project(costmap)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(costmap_lib
  src/costmap_core.cpp)
target_include_directories(costmap_lib
  PUBLIC include)
ament_target_dependencies(costmap_lib
  rclcpp
  sensor_msgs
  nav_msgs
)

add_executable(costmap_node src/costmap_node.cpp)
target_link_libraries(costmap_node costmap_lib)
ament_target_dependencies(costmap_node
  rclcpp
  sensor_msgs
  nav_msgs
)

install(TARGETS
  costmap_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
