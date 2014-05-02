cmake_minimum_required(VERSION 2.8.3)
project(mav_gazebo_plugins)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mav_model
  mav_control
  mav_msgs
  rosbag
)

find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

catkin_package(
 INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
 LIBRARIES mav_gazebo_motor_model mav_gazebo_controller_interface
 CATKIN_DEPENDS roscpp mav_model mav_control mav_msgs
 DEPENDS eigen gazebo
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_library(mav_gazebo_motor_model src/gazebo_motor_model.cpp)
target_link_libraries(mav_gazebo_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(mav_gazebo_controller_interface src/gazebo_controller_interface.cpp)
target_link_libraries(mav_gazebo_controller_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(
  mav_gazebo_controller_interface ${catkin_EXPORTED_TARGETS})

add_library(mav_gazebo_bag_plugin src/gazebo_bag_plugin.cpp)
target_link_libraries(mav_gazebo_bag_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(
  mav_gazebo_bag_plugin ${catkin_EXPORTED_TARGETS})
