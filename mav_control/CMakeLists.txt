cmake_minimum_required(VERSION 2.8.3)
project(mav_control)

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
)


add_library(mav_control
  src/controller_factory.cpp
  src/controller_base.cpp
  src/px4_dummy_controller.cpp
)

target_link_libraries(mav_control ${catkin_LIBRARIES})
add_dependencies(mav_control ${catkin_EXPORTED_TARGETS})
