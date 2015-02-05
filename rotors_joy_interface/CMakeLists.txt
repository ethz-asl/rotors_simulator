cmake_minimum_required(VERSION 2.8.3)
project(mav_joy_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mav_msgs
  sensor_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs mav_msgs sensor_msgs
  LIBRARIES
)

add_executable(mav_joy_interface src/joy.cpp)
add_dependencies(mav_joy_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(mav_joy_interface ${catkin_LIBRARIES})
