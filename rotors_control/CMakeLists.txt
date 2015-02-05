cmake_minimum_required(VERSION 2.8.3)
project(rotors_control)

add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES rotors_control
  CATKIN_DEPENDS geometry_msgs mav_msgs roscpp sensor_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(rotors_control
  src/controller_factory.cpp
  src/controller_base.cpp
  src/attitude_controller.cpp
  src/rate_controller.cpp
  src/attitude_controller_samy.cpp
  src/motor_controller.cpp
  src/lee_position_controller.cpp
)

target_link_libraries(rotors_control ${catkin_LIBRARIES})
add_dependencies(mrotors_control ${catkin_EXPORTED_TARGETS})

add_executable(ros_controller_node src/ros_controller_interface.cpp)
add_dependencies(ros_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_controller_node rotors_control ${catkin_LIBRARIES})
