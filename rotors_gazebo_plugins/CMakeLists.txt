# Optional arguments to be passed into file
# ADDITIONAL_INCLUDE_DIRS           string  Additional include directories to add to every build target (PX4 uses this).
# BUILD_MAVLINK_TOGGLE              bool    Enable or disable mavlink plugin
# BUILD_MAVLINK_INTERFACE_PLUGIN    bool    Build mavlink_interface_plugin if mavlink found (requires mav dependency).
# BUILD_OCTOMAP_PLUGIN              bool    Build the optical map plugin (requires Octomap).
# BUILD_OPTICAL_FLOW_PLUGIN         bool    Build the optical flow plugin (requires OpenCV).
# MAVLINK_HEADER_DIR                string  Location of MAVLink header files. If not provided, this CMakeLists.txt file will
#                                               search the default locations (e.g. ROS) for them. This variable is only required
#                                               if BUILD_MAVLINK_INTERFACE_PLUGIN=TRUE.
# NO_ROS                            bool    Build without any ROS dependencies.

cmake_minimum_required(VERSION 2.8.3)
project(rotors_gazebo_plugins)

#
#if ( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
#  message( FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt." )
#endif()

#message(STATUS "rotors_gazebo_plugins CMakeLists.txt called.")

# =============================================================================================== #
# ========================== SET DEFAULTS FOR PASSED-IN VARIABLES =============================== #
# =============================================================================================== #

set(BUILD_MAVLINK_TOGGLE TRUE) # Set to TRUE to build MAVLINK plugin

if(${BUILD_MAVLINK_TOGGLE})
  find_package(mavlink QUIET)
  if(${mavlink_FOUND})
    message(STATUS " mavlink found, building MAVLINK_INTERFACE_PLUGIN.")
    set(BUILD_MAVLINK_INTERFACE_PLUGIN TRUE)
  else()
    message(STATUS " mavlink not found, not building MAVLINK_INTERFACE_PLUGIN.")
    set(BUILD_MAVLINK_INTERFACE_PLUGIN FALSE)
  endif()
else()
  message(STATUS " mavlink toggle off, not building MAVLINK_INTERFACE_PLUGIN.")
  set(BUILD_MAVLINK_INTERFACE_PLUGIN FALSE)
endif()

if(NOT DEFINED BUILD_OCTOMAP_PLUGIN)
  message(STATUS "BUILD_OCTOMAP_PLUGIN variable not provided, setting to FALSE.")
  set(BUILD_OCTOMAP_PLUGIN FALSE)
endif()

if(NOT DEFINED BUILD_OPTICAL_FLOW_PLUGIN)
  message(STATUS "BUILD_OPTICAL_FLOW_PLUGIN variable not provided, setting to FALSE.")
  set(BUILD_OPTICAL_FLOW_PLUGIN FALSE)
endif()

if(NOT DEFINED NO_ROS)
  message(STATUS "NO_ROS variable not provided, setting to FALSE.")
  set(NO_ROS FALSE)
endif()

# Add any additional include directories as specified by the calling process (either user or another CMake file).
# ASL: Doesn't use this, catkin manages the mav_comm dependency
# PX4: Provides include directory for mav_msgs, so that "mav_msgs/default_topics.h" can be found and used.
include_directories(${ADDITIONAL_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

# Initialise a list which will keep track of all targets
# that need to be installed.
set(targets_to_install "")

# =============================================================================================== #
# ======================================== STATUS MESSAGES ====================================== #
# =============================================================================================== #

message(STATUS "ADDITIONAL_INCLUDE_DIRS = ${ADDITIONAL_INCLUDE_DIRS}")

if(BUILD_OCTOMAP_PLUGIN)
  message(STATUS "BUILD_OCTOMAP_PLUGIN = TRUE, building gazebo_octomap_plugin.")
else ()
  message(STATUS "BUILD_OCTOMAP_PLUGIN = FALSE, NOT building gazebo_octomap_plugin.")
endif ()

if(BUILD_OPTICAL_FLOW_PLUGIN)
  message(STATUS "BUILD_OPTICAL_FLOW_PLUGIN = TRUE, building gazebo_optical_flow_plugin.")
else ()
  message(STATUS "BUILD_OPTICAL_FLOW_PLUGIN = FALSE, NOT building gazebo_optical_flow_plugin.")
endif ()

if(NO_ROS)
  message(STATUS "NO_ROS = TRUE, building rotors_gazebo_plugins WITHOUT any ROS dependancies.")
else()
  message(STATUS "NO_ROS = FALSE, building rotors_gazebo_plugins WITH ROS dependancies.")
endif()

# Specify C++11 standard
add_definitions(-std=c++11)

# Provides a compiler flag notifying the preprocessor about
# the MAVLink Interface plugin build status
add_definitions(
  -DMAVLINK_INTERFACE=${BUILD_MAVLINK_INTERFACE_PLUGIN}
)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

if (NOT NO_ROS)
  find_package(catkin REQUIRED COMPONENTS
    gazebo_plugins
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

message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}" )

find_package(Eigen REQUIRED)
find_package(gazebo_dev QUIET)
if (NOT GAZEBO_DEV_FOUND)
  find_package(gazebo REQUIRED)
endif()

if(${gazebo_VERSION_MAJOR} LESS 9)
  if(${gazebo_VERSION_MAJOR} LESS 7)
    message(WARN "GAZEBO Version ${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}.${gazebo_VERSION_PATCH}")
    message(WARN "GAZEBO older than v9 detected. Autobackporting source files")

    execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/autobackport/commandpre7.sh" "" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" RESULT_VARIABLE git_result)

  else()
    message(WARN "GAZEBO Version ${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}.${gazebo_VERSION_PATCH}")
    message(WARN "GAZEBO older than v9 detected. Autobackporting source files")

    execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/autobackport/command.sh" "" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" RESULT_VARIABLE git_result)
  endif()

endif()

find_package(OpenCV REQUIRED)
find_package(Glog REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


if (BUILD_MAVLINK_INTERFACE_PLUGIN)
  include_directories(${mavros_msgs_INCLUDE_DIRS})
  include_directories(${libmavconn_INCLUDE_DIRS})
endif()

# ============================================== #
# =================== YAML_CPP ================= #
# ============================================== #

find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
  message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
  set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
  set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
  message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
  pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()
include_directories(${YamlCpp_INCLUDE_DIR})

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

# Get lists of all .proto files in the msgs directory
file(GLOB msgs msgs/*.proto)

# Finally, generate the .cpp files from the .proto files
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

# Create a shared library of protobuf messages (.so extension on Linux platforms)
add_library(mav_msgs SHARED ${PROTO_SRCS})
target_link_libraries(mav_msgs ${PROTOBUF_LIBRARY} gazebo_msgs)

# This causes mav_msgs to be linked with every created library in this file from this
# point forward.
# NOTE: This is deprecated, should be using target_link_libraries instead
link_libraries(mav_msgs)

# ============================================== #
# ==================== CATKIN ================== #
# ============================================== #

if (NOT NO_ROS)

  if(${gazebo_dev_FOUND})
    catkin_package(
      INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
      LIBRARIES rotors_gazebo_motor_model rotors_gazebo_controller_interface
      CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs octomap_msgs octomap_ros rosbag roscpp gazebo_plugins rotors_comm rotors_control std_srvs tf
      DEPENDS Eigen octomap OpenCV
      #CFG_EXTRAS rotors_gazebo_plugins.cmake
    )
    set(target_linking_LIBRARIES ${catkin_LIBRARIES} ${gazebo_dev_LIBRARIES})
  else()
    catkin_package(
      INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
      LIBRARIES rotors_gazebo_motor_model rotors_gazebo_controller_interface
      CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs octomap_msgs octomap_ros rosbag roscpp gazebo_plugins rotors_comm rotors_control std_srvs tf
      DEPENDS Eigen octomap OpenCV
      #CFG_EXTRAS rotors_gazebo_plugins.cmake
    )
    set(target_linking_LIBRARIES ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  endif()
endif()


# Including GAZEBO_MSG_INCLUDE_DIRS here allows the .cpp files generated by custom .proto files to find
# the .cpp/hpp files generated by the built-in Gazebo .proto files
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_MSG_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})


# =============================================================================================== #
# ========================================= USER LIBRARIES ====================================== #
# =============================================================================================== #

# SORTED IN ALPHABETICAL ORDER (by "plugin" name, keep it this way!)

#========================================= BAG PLUGIN ===========================================//
# Entire GazeboBagPlugin is a heavy ROS dependency, and so rather than passing messages to
# GazeboRosInterfacePlugin, this entire library is only included if ROS is present.
if (NOT NO_ROS)
  add_library(rotors_gazebo_bag_plugin SHARED src/gazebo_bag_plugin.cpp)
  target_link_libraries(rotors_gazebo_bag_plugin ${target_linking_LIBRARIES} )
  add_dependencies(rotors_gazebo_bag_plugin ${catkin_EXPORTED_TARGETS})
  list(APPEND targets_to_install rotors_gazebo_bag_plugin)
endif()

#================================= CONTROLLER INTERFACE PLUGIN ==================================//
add_library(rotors_gazebo_controller_interface SHARED src/gazebo_controller_interface.cpp)
target_link_libraries(rotors_gazebo_controller_interface ${target_linking_LIBRARIES} )
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_controller_interface ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_controller_interface)

#=================================== GEOTAGGED IMAGES PLUGIN ====================================//

message(STATUS "GAZEBO ${gazebo_VERSION_MAJOR}.${gazebo_VERSION_MINOR}.${gazebo_VERSION_PATCH}")
message(STATUS "${GAZEBO_VERSION_MAJOR}")

if(${gazebo_VERSION_MAJOR} GREATER 4)
  add_library(gazebo_geotagged_images_plugin SHARED src/external/gazebo_geotagged_images_plugin.cpp)
  target_link_libraries(gazebo_geotagged_images_plugin ${target_linking_LIBRARIES} )
  if (NOT NO_ROS)
    add_dependencies(gazebo_geotagged_images_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install gazebo_geotagged_images_plugin)
else()
  message(STATUS "Gazebo version is less than 5, not building gazebo_geotagged_images_plugin.cpp.")
endif()

#===================================== FW DYNAMICS PLUGIN =======================================//
add_library(rotors_gazebo_fw_dynamics_plugin SHARED src/gazebo_fw_dynamics_plugin.cpp)
target_link_libraries(rotors_gazebo_fw_dynamics_plugin ${target_linking_LIBRARIES}  ${YamlCpp_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_fw_dynamics_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_fw_dynamics_plugin)

#========================================= GPS PLUGIN ===========================================//
add_library(rotors_gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)
target_link_libraries(rotors_gazebo_gps_plugin ${target_linking_LIBRARIES} )
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_gps_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_gps_plugin)

#========================================= IMU PLUGIN ===========================================//
add_library(rotors_gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
target_link_libraries(rotors_gazebo_imu_plugin ${target_linking_LIBRARIES} )
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_imu_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_imu_plugin)

#========================================= NoisyDepth PLUGIN ======================================//
if(${gazebo_dev_FOUND})
  add_library(rotors_gazebo_noisydepth_plugin SHARED src/gazebo_noisydepth_plugin.cpp
          src/depth_noise_model.cpp)

  # As the depth camera plugins .so's are not part of any cmake accessible library,
  # but the plugin path is path is, we iterate over all paths until plugin folder is found.
  foreach(subdir ${gazebo_dev_LIBRARY_DIRS})
    if (${subdir} MATCHES "gazebo.*plugins")
      set(gazebo_dev_DEPTHCAMERA_LIB ${subdir}/libDepthCameraPlugin${CMAKE_SHARED_LIBRARY_SUFFIX})
    endif()
  endforeach()

  target_link_libraries(rotors_gazebo_noisydepth_plugin ${target_linking_LIBRARIES} ${gazebo_dev_DEPTHCAMERA_LIB})
  if (NOT NO_ROS)
    add_dependencies(rotors_gazebo_noisydepth_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install rotors_gazebo_noisydepth_plugin)
else()
  message(STATUS "Gazebo version too old (no gazebo_dev package), not building gazebo_noisydepth_plugin.cpp.")
endif()

#======================================== LIDAR PLUGIN ==========================================//
if(${gazebo_VERSION_MAJOR} GREATER 4)
  add_library(rotors_gazebo_lidar_plugin SHARED src/external/gazebo_lidar_plugin.cpp)
  target_link_libraries(rotors_gazebo_lidar_plugin ${target_linking_LIBRARIES} )
  if (NOT NO_ROS)
    add_dependencies(rotors_gazebo_lidar_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install rotors_gazebo_lidar_plugin)
else()
  message(STATUS "Gazebo version is less than 5, not building gazebo_lidar_plugin.cpp.")
endif()

#===================================== MAGNETOMETER PLUGIN ======================================//
add_library(rotors_gazebo_magnetometer_plugin SHARED src/gazebo_magnetometer_plugin.cpp)
target_link_libraries(rotors_gazebo_magnetometer_plugin ${target_linking_LIBRARIES} )
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_magnetometer_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_magnetometer_plugin)

#================================= MAVLINK INTERFACE PLUGIN =====================================//

if(BUILD_MAVLINK_INTERFACE_PLUGIN)
  message(STATUS "BUILD_MAVLINK_INTERFACE_PLUGIN = TRUE, adding mavros dependency and building mavlink_interface_plugin.")
else ()
  message(STATUS "BUILD_MAVLINK_INTERFACE_PLUGIN = FALSE, NOT adding mavros dependency and NOT building mavlink_interface_plugin.")
endif ()

if (BUILD_MAVLINK_INTERFACE_PLUGIN)
  # needed to ensure mavros is actually built by the time we call find_package
  add_custom_target("mavros_dummy_target" DEPENDS mavros)

  find_package(mavros QUIET)

  # We need the MAVLink headers.
  set(MAVLINK_HEADERS_FOUND FALSE)
  set(MAVROS_VALID FALSE)

  # First, check to see if MAVLink headers were passed in as variable
  if(NOT ${MAVLINK_HEADER_DIR} STREQUAL "")
    message(STATUS "MAVLINK_HEADER_DIR provided as '${MAVLINK_HEADER_DIR}'.")
    include_directories(${MAVLINK_HEADER_DIR})
    set(MAVLINK_HEADERS_FOUND TRUE)
  else()
    message(WARN "MAVLINK_HEADER_DIR not provided, falling back to looking at default paths...")
    # fist check if there's a mavlink dir in the current workspace
    if(EXISTS ${mavlink_INCLUDE_DIRS}/mavlink/v2.0)
      message(STATUS "Found MAVLink headers in workspace at ' ${mavlink_INCLUDE_DIRS}/mavlink/v2.0/'.")
      include_directories("${mavlink_INCLUDE_DIRS}/mavlink/v2.0/")
      set(MAVLINK_HEADERS_FOUND TRUE)
    # If ROS is installed, we should be able to find them at the path below
    elseif(EXISTS /opt/ros/$ENV{ROS_DISTRO}/include/mavlink/v2.0/)
      message(STATUS "Found MAVLink headers at '/opt/ros/$ENV{ROS_DISTRO}/include/mavlink/v2.0/'.")
      include_directories("/opt/ros/$ENV{ROS_DISTRO}/include/mavlink/v2.0/")
      set(MAVLINK_HEADERS_FOUND TRUE)
    endif()

  endif()

  # Check mavros version
  if(mavros_VERSION_MAJOR GREATER 0 OR mavros_VERSION_MINOR GREATER 17)
    set(MAVROS_VALID TRUE)
  else()
    message(WARN "MAVROS version too old (${mavros_VERSION}) - need version newer than 0.17, not building MavlinkInterfacePlugin")
  endif()

  if(NOT MAVLINK_HEADERS_FOUND)
    message(ERROR "MAVLink headers were not found. They are required for building MavlinkInterfacePlugin. Not building MavlinkInterfacePlugin.")
  endif()

  if(MAVLINK_HEADERS_FOUND AND MAVROS_VALID)
   message(WARN "Mavlink headers found and mavros version check successful, building MavlinkInterfacePlugin")
   # Note that this library includes TWO .cpp files.
   add_library(rotors_gazebo_mavlink_interface SHARED src/gazebo_mavlink_interface.cpp src/geo_mag_declination.cpp)
   target_link_libraries(rotors_gazebo_mavlink_interface ${target_linking_LIBRARIES}  ${mav_msgs})
   add_dependencies(rotors_gazebo_mavlink_interface ${catkin_EXPORTED_TARGETS} ${mavros_EXPORTED_TARGETS} ${mavros_msgs_EXPORTED_TARGETS})
   list(APPEND targets_to_install rotors_gazebo_mavlink_interface)
  endif()


endif()

#==================================== MOTOR MODEL PLUGIN ========================================//
add_library(rotors_gazebo_motor_model SHARED src/gazebo_motor_model.cpp)
target_link_libraries(rotors_gazebo_motor_model ${target_linking_LIBRARIES} )
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_motor_model ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_motor_model)

#==================================== MULTIROTOR BASE PLUGIN ====================================//
add_library(rotors_gazebo_multirotor_base_plugin SHARED src/gazebo_multirotor_base_plugin.cpp)
target_link_libraries(rotors_gazebo_multirotor_base_plugin ${target_linking_LIBRARIES} )
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_multirotor_base_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_multirotor_base_plugin)

#====================================== OCTOMAP PLUGIN ==========================================//

# Conditionally built since it requires Octomap as a dependency
# ASL uses this, PX4 does not
if(BUILD_OCTOMAP_PLUGIN)
  find_package(octomap REQUIRED)
  add_library(rotors_gazebo_octomap_plugin SHARED src/gazebo_octomap_plugin.cpp)
  target_link_libraries(rotors_gazebo_octomap_plugin ${target_linking_LIBRARIES} )
  if (NOT NO_ROS)
    add_dependencies(rotors_gazebo_octomap_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install rotors_gazebo_octomap_plugin)
endif()

#======================================= ODOMETRY PLUGIN ========================================//
add_library(rotors_gazebo_odometry_plugin SHARED src/gazebo_odometry_plugin.cpp)
target_link_libraries(rotors_gazebo_odometry_plugin ${target_linking_LIBRARIES}  ${OpenCV_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_odometry_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_odometry_plugin)

#===================================== OPTICAL FLOW PLUGIN ======================================//
# Since the optical flow plugin depends on external code (PX4/OpticalFlow), this is
# only conditionally built
if(BUILD_OPTICAL_FLOW_PLUGIN)

  # Check to make sure the correct Gazebo version is available.
  if(${gazebo_VERSION_MAJOR} LESS 5)
    message(FATAL_ERROR "Gazebo version needs to be >= v5.x. You specified BUILD_OPTICAL_FLOW_PLUGIN=TRUE, but Gazebo version was less than v5.x.")
  endif()

  find_package(Boost 1.40.0 REQUIRED timer)

  # OpticalFlow needs to be built and installed by the calling script, as it is very
  # unlikely that this package already exists on the users computer.
  # In PX4 this is done with a call to ExternalProject_Add().
  find_package(OpticalFlow REQUIRED)

  add_library(rotors_gazebo_optical_flow_plugin SHARED src/external/gazebo_optical_flow_plugin.cpp)
  target_include_directories(rotors_gazebo_optical_flow_plugin PUBLIC ${OpticalFlow_INCLUDE_DIRS})
  target_link_libraries(rotors_gazebo_optical_flow_plugin
    ${target_linking_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpticalFlow_LIBRARIES})
  if (NOT NO_ROS)
    add_dependencies(rotors_gazebo_optical_flow_plugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install rotors_gazebo_optical_flow_plugin)
endif()

#======================================= PRESSURE PLUGIN ========================================//
add_library(rotors_gazebo_pressure_plugin SHARED src/gazebo_pressure_plugin.cpp)
target_link_libraries(rotors_gazebo_pressure_plugin ${target_linking_LIBRARIES}  ${GLOG_LIBRARIES})
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_pressure_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_pressure_plugin)

#===================================== ROS INTERFACE PLUGIN =====================================//
# This entire plugin is only built if ROS is a dependency
if (NOT NO_ROS)
  add_library(rotors_gazebo_ros_interface_plugin SHARED src/gazebo_ros_interface_plugin.cpp)
  target_link_libraries(rotors_gazebo_ros_interface_plugin ${target_linking_LIBRARIES} )
  add_dependencies(rotors_gazebo_ros_interface_plugin ${catkin_EXPORTED_TARGETS})
  list(APPEND targets_to_install rotors_gazebo_ros_interface_plugin)
endif()

#========================================= WIND PLUGIN ==========================================//
add_library(rotors_gazebo_wind_plugin SHARED src/gazebo_wind_plugin.cpp)
target_link_libraries(rotors_gazebo_wind_plugin ${target_linking_LIBRARIES} )
if (NOT NO_ROS)
  add_dependencies(rotors_gazebo_wind_plugin ${catkin_EXPORTED_TARGETS})
endif()
list(APPEND targets_to_install rotors_gazebo_wind_plugin)

# =============================================================================================== #
# ======================================= EXTERNAL LIBRARIES ==================================== #
# =============================================================================================== #

# liftdrag_plugin is provided by Gazebo, but not guaranteed to be on system.
# Naming has not been changed to match rotors convetion, due to this being an external
# code soure.
# Linux is not consistent with plugin availability, even on Gazebo 7
#if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
if(${gazebo_VERSION_MAJOR} GREATER 4)
  add_library(LiftDragPlugin SHARED src/liftdrag_plugin/liftdrag_plugin.cpp)
  target_link_libraries(LiftDragPlugin ${target_linking_LIBRARIES} )
  if (NOT NO_ROS)
    add_dependencies(LiftDragPlugin ${catkin_EXPORTED_TARGETS})
  endif()
  list(APPEND targets_to_install LiftDragPlugin)
else()
  message(STATUS "Gazebo version is less than 5, not building liftdrag_plugin.cpp.")
endif()

message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
if (NOT NO_ROS)
  set(BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
else()
  set(BIN_DESTINATION ${CMAKE_BINARY_DIR})
  set(LIB_DESTINATION ${CMAKE_BINARY_DIR})
endif()

# Install all targets that were defined above (some conditionally) and added to the
# variable targets_to_install
install(
  TARGETS
    ${targets_to_install}
  DESTINATION ${BIN_DESTINATION}
  LIBRARY DESTINATION ${LIB_DESTINATION}
)

#message(FATAL_ERROR "Reached EOF.")
