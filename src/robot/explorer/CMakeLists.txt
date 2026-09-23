cmake_minimum_required(VERSION 3.10)
project(explorer)

# Set compiler to use C++ 17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)            # OccupancyGrid, Odometry, Path
find_package(geometry_msgs REQUIRED)       # PointStamped goals
find_package(std_msgs REQUIRED)            # Bool (on/off), Empty (cancel), String (status)
find_package(visualization_msgs REQUIRED)  # frontier markers for Foxglove

# The frontier search lives in a library so the unit tests can use it too
add_library(explorer_lib
  src/explorer_core.cpp)
target_include_directories(explorer_lib
  PUBLIC include)
ament_target_dependencies(explorer_lib
  rclcpp
  nav_msgs
)

add_executable(explorer_node src/explorer_node.cpp)
target_link_libraries(explorer_node explorer_lib)
ament_target_dependencies(explorer_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS
  explorer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

# Unit tests for the frontier search. Installed next to the node, so they can
# be run inside the robot image: ros2 run explorer explorer_test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(explorer_test test/explorer_core_test.cpp)
  target_link_libraries(explorer_test explorer_lib)
  install(TARGETS
    explorer_test
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
