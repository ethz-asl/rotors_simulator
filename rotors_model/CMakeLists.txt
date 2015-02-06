cmake_minimum_required(VERSION 2.8.3)
project(rotors_model)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
)
find_package(catkin REQUIRED)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
 INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} 
 LIBRARIES
 CATKIN_DEPENDS
 DEPENDS eigen
)
