cmake_minimum_required(VERSION 3.0.2)
project(move_box_plugin)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

# Find Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_box_plugin
  CATKIN_DEPENDS gazebo_ros roscpp
  DEPENDS system_lib
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(move_box_plugin src/MoveBoxPlugin.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(move_box_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

