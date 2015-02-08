cmake_minimum_required(VERSION 2.8.3)
project(rotors_joy_interface)

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

add_executable(rotors_joy_interface src/joy.cpp)
add_dependencies(rotors_joy_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotors_joy_interface ${catkin_LIBRARIES})
