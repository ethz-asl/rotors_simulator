cmake_minimum_required(VERSION 2.8.3)
project(mav_control)

set(PX4_DIR /home/roman/src/Firmware)
set ( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-D__EXPORT=")

add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES mav_control
  CATKIN_DEPENDS geometry_msgs mav_msgs roscpp sensor_msgs
  DEPENDS Eigen
)

include_directories(
  include
  include/mav_control
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PX4_DIR}/src
  ${PX4_DIR}/src/lib
  ${PX4_DIR}/src/modules
  ${PX4_DIR}/Tools/tests-host
)


add_library(mav_control
  src/controller_factory.cpp
  src/controller_base.cpp
  src/attitude_controller.cpp
  src/rate_controller.cpp
  src/attitude_controller_samy.cpp
  src/motor_controller.cpp
  src/lee_position_controller.cpp
  src/px4_att_controller.cpp
  src/mc_mixer.cpp
  src/perf_counter.cpp
  ${PX4_DIR}/src/modules/mc_att_control/mc_att_control_base.cpp
  ${PX4_DIR}/src/lib/mathlib/math/Limits.cpp
)

target_link_libraries(mav_control ${catkin_LIBRARIES})
add_dependencies(mav_control ${catkin_EXPORTED_TARGETS})
