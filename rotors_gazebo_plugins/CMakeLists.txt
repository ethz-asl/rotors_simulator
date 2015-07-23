cmake_minimum_required(VERSION 2.8.3)
project(rotors_gazebo_plugins)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  rotors_model
  rotors_control
  mav_msgs
  rosbag
  octomap_msgs
  planning_msgs
  geometry_msgs
  cmake_modules
  glog_catkin
  message_generation
  mavros
)

find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES rotors_gazebo_motor_model rotors_gazebo_controller_interface
  CATKIN_DEPENDS roscpp tf rotors_model mav_msgs octomap_ros octomap_msgs planning_msgs geometry_msgs glog_catkin
  DEPENDS eigen gazebo octomap opencv glog_catkin message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_library(rotors_gazebo_motor_model src/gazebo_motor_model.cpp)
target_link_libraries(rotors_gazebo_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

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
