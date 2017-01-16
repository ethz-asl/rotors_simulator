# Optional arguments to be passed into file
# NO_ROS: Build without any ROS dependencies
# MAVLINK_INTERFACE: Build mavlink_interface_plugin (requires mav dependency)

cmake_minimum_required(VERSION 2.8.3)
project(rotors_gazebo_plugins)
#
#if ( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
#  message( FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt." )
#endif()

message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

#set(MAVLINK_INTERFACE)
set(BUILD_MAVLINK_INTERFACE)

if(NO_ROS)
  message(STATUS "NO_ROS = TRUE, building rotors_gazebo_plugins WITHOUT any ROS dependancies.")
else()
  message(STATUS "NO_ROS = FALSE, building rotors_gazebo_plugins WITH ROS dependancies.")
endif()

#if (MAVLINK_INTERFACE STREQUAL "y")
if (MAVLINK_INTERFACE)
  set(BUILD_MAVLINK_INTERFACE TRUE)
  message(STATUS "MAVLINK_INTERFACE = TRUE, adding mavros dependency and building mavlink_interface_plugin.")
else ()
  set(BUILD_MAVLINK_INTERFACE FALSE)
  message(STATUS "MAVLINK_INTERFACE = FALSE, NOT adding mavros dependency and NOT building mavlink_interface_plugin.")
endif ()

# Specify C++11 standard
add_definitions(-std=c++11)

add_definitions(
  -DMAVLINK_INTERFACE=${MAVLINK_INTERFACE}
)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

if (NOT NO_ROS)
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
endif()

# if (BUILD_MAVLINK_INTERFACE)
#   find_package(mavros)
#   find_package(mavros_msgs)
#   find_package(libmavconn)
#   message(STATUS "Added mavros dependency")
# endif()

message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# if (BUILD_MAVLINK_INTERFACE)
#   include_directories(${mavros_msgs_INCLUDE_DIRS})
#   include_directories(${libmavconn_INCLUDE_DIRS})
# endif()

# ============================================== #
# =================== PROTOBUF ================= #
# ============================================== #

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()
message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

# protobuf required for publishing/subscribing to Gazebo
# messages
# WARNING: THIS MUST BE RUN AFTER PROTOBUF_IMPORT_DIRS IS SETUP
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

# Protobuf version check
if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

# After running this block of code,
# we should have something similar to 
# GAZEBO_MSG_INCLUDE_DIRS = /usr/include/gazebo-7/gazebo/msgs
set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()
#message(FATAL_ERROR "GAZEBO_MSG_INCLUDE_DIRS = " ${GAZEBO_MSG_INCLUDE_DIRS})

set (msgs
  msgs/Actuators.proto
  msgs/CommandMotorSpeed.proto
  msgs/ConnectGazeboToRosTopic.proto
  msgs/ConnectRosToGazeboTopic.proto
  msgs/Float32.proto
  msgs/Header.proto
  msgs/JointState.proto
  msgs/Lidar.proto
  msgs/MagneticField.proto
  msgs/MotorSpeed.proto
  msgs/NavSatFix.proto
  msgs/Odometry.proto
  msgs/OpticalFlow.proto
  msgs/Point.proto
  msgs/PointStamped.proto
  msgs/Pose.proto
  msgs/PoseStamped.proto
  msgs/PoseWithCovariance.proto
  msgs/PoseWithCovarianceStamped.proto
  msgs/Position.proto
  msgs/PositionStamped.proto
  msgs/Quaternion.proto
  msgs/SensorImu.proto
  msgs/Transform.proto
  msgs/TransformStamped.proto
  msgs/Twist.proto
  msgs/TwistStamped.proto
  msgs/TwistWithCovariance.proto
  msgs/Vector3.proto
  msgs/WindSpeed.proto
)

# Finally, generate the .cpp files from the .proto files
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

# Create a shared library of protobuf messages (.so extension on Linux platforms)
add_library(mav_msgs SHARED ${PROTO_SRCS})
target_link_libraries(mav_msgs ${PROTOBUF_LIBRARY})

# This causes mav_msgs to be linked with every created library in this file from this
# point forward.
# NOTE: This is deprecated, should be using target_link_libraries instead
link_libraries(mav_msgs)

# ============================================== #
# ==================== CATKIN ================== #
# ============================================== #

if (NOT NO_ROS)
  catkin_package(
    INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include/${PACKAGE_NAME}
    LIBRARIES rotors_gazebo_motor_model rotors_gazebo_controller_interface
    CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs octomap_msgs octomap_ros rosbag roscpp rotors_comm rotors_control std_srvs tf
    DEPENDS eigen gazebo octomap opencv
    #CFG_EXTRAS rotors_gazebo_plugins.cmake  
  )
endif()


# Including GAZEBO_MSG_INCLUDE_DIRS here allows the .cpp files generated by custom .proto files to find
# the .cpp/hpp files generated by the built-in Gazebo .proto files
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_MSG_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

# ============================================== #
# ============ LIBRARIES IN PACKAGE ============ #
# ============================================== #

# SORTED IN ALPHABETICAL ORDER (by "plugin" name, keep it this way!)

#========================================= BAG PLUGIN ===========================================//
# Entire GazeboBagPlugin is a heavy ROS dependency, and so rather than passing messages to
# GazeboRosInterfacePlugin, this entire library is only included if ROS is present.
if (NOT NO_ROS)
  add_library(rotors_gazebo_bag_plugin src/gazebo_bag_plugin.cpp)
  target_link_libraries(rotors_gazebo_bag_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  add_dependencies(rotors_gazebo_bag_plugin ${catkin_EXPORTED_TARGETS})
endif()

#================================= CONTROLLER INTERFACE PLUGIN ==================================//
add_library(rotors_gazebo_controller_interface src/gazebo_controller_interface.cpp)
target_link_libraries(rotors_gazebo_controller_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_controller_interface ${catkin_EXPORTED_TARGETS})
endif()

#=================================== GEOTAGGED IMAGES PLUGIN ====================================//
add_library(gazebo_geotagged_images_plugin src/gazebo_geotagged_images_plugin.cpp)
target_link_libraries(gazebo_geotagged_images_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(gazebo_geotagged_images_plugin ${catkin_EXPORTED_TARGETS})
endif()

#========================================= GPS PLUGIN ===========================================//
add_library(rotors_gazebo_gps_plugin src/gazebo_gps_plugin.cpp)
target_link_libraries(rotors_gazebo_gps_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_gps_plugin ${catkin_EXPORTED_TARGETS})
endif()

#========================================= IMU PLUGIN ===========================================//
add_library(rotors_gazebo_imu_plugin src/gazebo_imu_plugin.cpp)
target_link_libraries(rotors_gazebo_imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_imu_plugin ${catkin_EXPORTED_TARGETS})
endif()

#======================================== LIDAR PLUGIN ==========================================//
add_library(rotors_gazebo_lidar_plugin SHARED src/gazebo_lidar_plugin.cpp)
target_link_libraries(rotors_gazebo_lidar_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_lidar_plugin ${catkin_EXPORTED_TARGETS})
endif()

#===================================== MAGNETOMETER PLUGIN ======================================//
add_library(rotors_gazebo_magnetometer_plugin src/gazebo_magnetometer_plugin.cpp)
target_link_libraries(rotors_gazebo_magnetometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_magnetometer_plugin ${catkin_EXPORTED_TARGETS})
endif()

#================================= MAVLINK INTERFACE PLUGIN =====================================//
if (BUILD_MAVLINK_INTERFACE)
  # Should this be SHARED or not???
  # Note that this library includes TWO .cpp files.
  add_library(rotors_gazebo_mavlink_interface SHARED src/gazebo_mavlink_interface.cpp src/geo_mag_declination.cpp)
  target_link_libraries(rotors_gazebo_mavlink_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${mav_msgs})
  #add_dependencies(rotors_gazebo_mavlink_interface ${catkin_EXPORTED_TARGETS} ${mavros_EXPORTED_TARGETS} ${mavros_msgs_EXPORTED_TARGETS})
  message(STATUS "Built mavlink_interface_plugin")
endif()

#==================================== MOTOR MODEL PLUGIN ========================================//
add_library(rotors_gazebo_motor_model src/gazebo_motor_model.cpp)
target_link_libraries(rotors_gazebo_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_motor_model ${catkin_EXPORTED_TARGETS})
endif()

#==================================== MULTIROTOR BASE PLUGIN ====================================//
add_library(rotors_gazebo_multirotor_base_plugin src/gazebo_multirotor_base_plugin.cpp)
target_link_libraries(rotors_gazebo_multirotor_base_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_multirotor_base_plugin ${catkin_EXPORTED_TARGETS})
endif()

#====================================== OCTOMAP PLUGIN ==========================================//
add_library(rotors_gazebo_octomap_plugin src/gazebo_octomap_plugin.cpp)
target_link_libraries(rotors_gazebo_octomap_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_octomap_plugin ${catkin_EXPORTED_TARGETS})
endif()

#======================================= ODOMETRY PLUGIN ========================================//
add_library(rotors_gazebo_odometry_plugin src/gazebo_odometry_plugin.cpp)
target_link_libraries(rotors_gazebo_odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_odometry_plugin ${catkin_EXPORTED_TARGETS})
endif()

#===================================== OPTICAL FLOW PLUGIN ======================================//
# Since the optical flow plugin depends on external code (PX4/OpticalFlow), this is
# only conditionally built
if(BUILD_OPTICAL_FLOW_PLUGIN)
  # TODO The path to the optical flow repo should not be referenced here!
  # (pass in path as variable instead?)
  find_package(OpticalFlow REQUIRED)
  message(STATUS "OpticalFlow_INCLUDE_DIRS = ${OpticalFlow_INCLUDE_DIRS}")
  #set(OpticalFlow_LIBS "OpticalFlow")
  
  add_library(rotors_gazebo_optical_flow_plugin SHARED src/gazebo_optical_flow_plugin.cpp)
  target_link_libraries(rotors_gazebo_optical_flow_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  if (NOT NO_ROS)
    add_dependencies(rotors_gazebo_optical_flow_plugin ${catkin_EXPORTED_TARGETS})
  endif()
endif()

#===================================== ROS INTERFACE PLUGIN =====================================//
# This entire plugin is only built if ROS is a dependency
if (NOT NO_ROS)
  add_library(rotors_gazebo_ros_interface_plugin src/gazebo_ros_interface_plugin.cpp)
  target_link_libraries(rotors_gazebo_ros_interface_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  add_dependencies(rotors_gazebo_ros_interface_plugin ${catkin_EXPORTED_TARGETS})
endif()

#========================================= WIND PLUGIN ==========================================//
add_library(rotors_gazebo_wind_plugin src/gazebo_wind_plugin.cpp)
target_link_libraries(rotors_gazebo_wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_wind_plugin ${catkin_EXPORTED_TARGETS})
endif()




message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
if (NOT NO_ROS)
  set(BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
else()
  set(BIN_DESTINATION ${CMAKE_BINARY_DIR})
  set(LIB_DESTINATION ${CMAKE_BINARY_DIR})
endif()


install(
  TARGETS
    rotors_gazebo_controller_interface
    gazebo_geotagged_images_plugin
    rotors_gazebo_gps_plugin
    rotors_gazebo_imu_plugin
    rotors_gazebo_lidar_plugin
    rotors_gazebo_magnetometer_plugin
    rotors_gazebo_motor_model
    rotors_gazebo_multirotor_base_plugin
    rotors_gazebo_octomap_plugin
    rotors_gazebo_odometry_plugin
    rotors_gazebo_wind_plugin
    
  DESTINATION ${BIN_DESTINATION}
  LIBRARY DESTINATION ${LIB_DESTINATION}
)

if (NOT NO_ROS)
    install(
      TARGETS
        rotors_gazebo_bag_plugin
      DESTINATION ${BIN_DESTINATION}
      LIBRARY DESTINATION ${LIB_DESTINATION}
    )
endif()

if (BUILD_MAVLINK_INTERFACE)
  install(
    TARGETS
      rotors_gazebo_mavlink_interface
    DESTINATION ${BIN_DESTINATION}
    LIBRARY DESTINATION ${LIB_DESTINATION}
  )
endif()

if(BUILD_OPTICAL_FLOW_PLUGIN)
  install(
    TARGETS
      rotors_gazebo_optical_flow_plugin
    DESTINATION ${BIN_DESTINATION}
    LIBRARY DESTINATION ${LIB_DESTINATION}
  )
endif()

# Copy over all converted protobuf files (*.pb.h and *.pb.c) from the src directory to
# the devel directory. This is required so that you can include the protobuf header files from other C++ files.
if (NOT NO_ROS)
  file(COPY ${CMAKE_CURRENT_BINARY_DIR}/ DESTINATION ${CATKIN_DEVEL_PREFIX}/include/${PACKAGE_NAME} FILES_MATCHING PATTERN "*.pb.*")
endif()

#message(FATAL_ERROR "Reached EOF.")
