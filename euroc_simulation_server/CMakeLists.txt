cmake_minimum_required(VERSION 2.8.3)
project(euroc_simulation_server)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  euroc_comm
  roscpp
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(  
  CATKIN_DEPENDS 
    geometry_msgs 
    mav_msgs 
    euroc_comm
    roscpp 
    sensor_msgs 
  DEPENDS Eigen
)
  
add_subdirectory(src/t1)
add_subdirectory(src/t2)
add_subdirectory(src/t3)
add_subdirectory(src/t4)
