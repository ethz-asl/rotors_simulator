cmake_minimum_required(VERSION 2.8.3)
project(mav_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mav_msgs
  roscpp
  sensor_msgs
  sensor_fusion_comm
)

find_package(Eigen REQUIRED)



catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES mav_control
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs mav_msgs roscpp sensor_msgs sensor_fusion_comm
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(mav_control
  src/controller_factory.cpp
  src/controller_base.cpp
  src/attitude_controller.cpp
  src/attitude_controller_samy.cpp
  src/motor_controller.cpp
)

target_link_libraries(mav_control ${catkin_LIBRARIES})

add_executable(ros_controller_node src/ros_controller_interface.cpp)

add_dependencies(ros_controller_node mav_control_generate_messages_cpp)
add_dependencies(ros_controller_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_controller_node mav_control ${catkin_LIBRARIES})
