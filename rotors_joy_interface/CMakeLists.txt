cmake_minimum_required(VERSION 2.8.3)
project(rotors_joy_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mav_msgs
  sensor_msgs
  trajectory_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs mav_msgs sensor_msgs trajectory_msgs
  LIBRARIES
)

add_definitions(-std=c++11)

add_executable(rotors_joy_interface src/joy.cpp)
add_dependencies(rotors_joy_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotors_joy_interface ${catkin_LIBRARIES})

install(TARGETS rotors_joy_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
