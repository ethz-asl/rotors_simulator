cmake_minimum_required(VERSION 2.8.3)
project(rotors_gazebo_plugins)

set(MAVLINK_INTERFACE)
set(BUILD_MAVLINK_INTERFACE)

if (MAVLINK_INTERFACE STREQUAL "y")
  set(BUILD_MAVLINK_INTERFACE TRUE)
else ()
  set(BUILD_MAVLINK_INTERFACE FALSE)
endif ()

if (BUILD_MAVLINK_INTERFACE)
  message(STATUS "Adding mavros dependency and building mavlink_interface_plugin")
endif ()

add_definitions(-std=c++11)

add_definitions(
  -DMAVLINK_INTERFACE=${MAVLINK_INTERFACE}
)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  mav_msgs
  octomap_msgs
  octomap_ros
  rosbag
  roscpp
  rotors_comm
  rotors_control
  std_srvs
  tf
)

if (BUILD_MAVLINK_INTERFACE)
  find_package(mavros)
  find_package(mavros_msgs)
  find_package(libmavconn)
  message(STATUS "Added mavros dependency")
endif()


find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
if (BUILD_MAVLINK_INTERFACE)
  include_directories(${mavros_msgs_INCLUDE_DIRS})
  include_directories(${libmavconn_INCLUDE_DIRS})
endif()

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES rotors_gazebo_motor_model rotors_gazebo_controller_interface
  CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs octomap_msgs octomap_ros rosbag roscpp rotors_comm rotors_control std_srvs tf
  DEPENDS eigen gazebo octomap opencv
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_library(rotors_gazebo_motor_model src/gazebo_motor_model.cpp)
target_link_libraries(rotors_gazebo_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_motor_model ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_odometry_plugin src/gazebo_odometry_plugin.cpp)
target_link_libraries(rotors_gazebo_odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(rotors_gazebo_odometry_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_imu_plugin src/gazebo_imu_plugin.cpp)
target_link_libraries(rotors_gazebo_imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_imu_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_controller_interface src/gazebo_controller_interface.cpp)
target_link_libraries(rotors_gazebo_controller_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_controller_interface ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_bag_plugin src/gazebo_bag_plugin.cpp)
target_link_libraries(rotors_gazebo_bag_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_bag_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_wind_plugin src/gazebo_wind_plugin.cpp)
target_link_libraries(rotors_gazebo_wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_wind_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_octomap_plugin src/gazebo_octomap_plugin.cpp)
target_link_libraries(rotors_gazebo_octomap_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_octomap_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_multirotor_base_plugin src/gazebo_multirotor_base_plugin.cpp)
target_link_libraries(rotors_gazebo_multirotor_base_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_multirotor_base_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_gps_plugin src/gazebo_gps_plugin.cpp)
target_link_libraries(rotors_gazebo_gps_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_gps_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_magnetometer_plugin src/gazebo_magnetometer_plugin.cpp)
target_link_libraries(rotors_gazebo_magnetometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_magnetometer_plugin ${catkin_EXPORTED_TARGETS})

if (BUILD_MAVLINK_INTERFACE)
  add_library(rotors_gazebo_mavlink_interface src/gazebo_mavlink_interface.cpp)
  target_link_libraries(rotors_gazebo_mavlink_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${mavros_LIBRARIES})
  add_dependencies(rotors_gazebo_mavlink_interface ${catkin_EXPORTED_TARGETS} ${mavros_EXPORTED_TARGETS} ${mavros_msgs_EXPORTED_TARGETS})
  message(STATUS "Built mavlink_interface_plugin")
endif()

install(
  TARGETS
    rotors_gazebo_motor_model
    rotors_gazebo_odometry_plugin
    rotors_gazebo_imu_plugin
    rotors_gazebo_controller_interface
    rotors_gazebo_bag_plugin
    rotors_gazebo_wind_plugin
    rotors_gazebo_octomap_plugin
    rotors_gazebo_multirotor_base_plugin
    rotors_gazebo_gps_plugin
    rotors_gazebo_magnetometer_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

if (BUILD_MAVLINK_INTERFACE)
  install(
    TARGETS
      rotors_gazebo_mavlink_interface
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endif()

