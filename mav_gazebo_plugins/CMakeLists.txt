cmake_minimum_required(VERSION 2.8.3)
project(mav_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mav_model
)

find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

catkin_package(
 INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
 LIBRARIES mav_gazebo
 CATKIN_DEPENDS roscpp mav_model
 DEPENDS eigen gazebo
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_library(mav_gazebo_motor_model src/gazebo_motor_model.cpp)
