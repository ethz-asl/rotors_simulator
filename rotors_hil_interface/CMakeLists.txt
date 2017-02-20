cmake_minimum_required(VERSION 2.8.3)
project(rotors_hil_interface)

add_definitions(-std=c++11)

find_package(Mavlink QUIET)

if (Mavlink_DIR)
  message(STATUS "Building HIL_INTERFACE package.")
else ()
  message(STATUS "Mavlink not found. Skipping HIL_INTERFACE package.")

  # We still have to call catkin package for this to be a valid package,
  # even if empty.
  find_package(catkin REQUIRED)
  catkin_package()
  return()
endif ()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  mav_msgs
  mavros
  mavros_msgs
  roscpp
  sensor_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rotors_hil_interface
  CATKIN_DEPENDS geometry_msgs mav_msgs mavros mavros_msgs roscpp sensor_msgs
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
)

add_library(rotors_hil_interface
  src/hil_sensor_level_interface.cpp
  src/hil_state_level_interface.cpp
)

target_link_libraries(rotors_hil_interface ${catkin_LIBRARIES} ${mavros_LIBRARIES})
add_dependencies(rotors_hil_interface ${catkin_EXPORTED_TARGETS} ${mavros_EXPORTED_TARGETS} ${mavros_msgs_EXPORTED_TARGETS})

add_executable(rotors_hil_interface_node src/hil_interface_node.cpp)
add_dependencies(rotors_hil_interface_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(rotors_hil_interface_node rotors_hil_interface ${catkin_LIBRARIES})

install(TARGETS
    rotors_hil_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
    rotors_hil_interface_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
