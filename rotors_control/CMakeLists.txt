cmake_minimum_required(VERSION 2.8.3)
project(rotors_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  manipulator_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
	message_filters
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES lee_position_controller roll_pitch_yawrate_thrust_controller multi_objective_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs manipulator_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(lee_position_controller
  src/library/lee_position_controller.cpp
)

add_library(roll_pitch_yawrate_thrust_controller
  src/library/roll_pitch_yawrate_thrust_controller.cpp
)

set(MATLAB_LIB_DIR include/${PROJECT_NAME}/dynamic_terms_fun)
set(MATLAB_LIB
		${MATLAB_LIB_DIR}/q34_12_fun/q34_12_fun.cpp
		${MATLAB_LIB_DIR}/M_triu_fun/M_triu_fun.cpp
		${MATLAB_LIB_DIR}/G_fun/G_fun.cpp
		${MATLAB_LIB_DIR}/C_fun/C_fun.cpp
		${MATLAB_LIB_DIR}/G_fun/rt_nonfinite.cpp
		${MATLAB_LIB_DIR}/G_fun/rtGetNaN.cpp
		${MATLAB_LIB_DIR}/G_fun/rtGetInf.cpp )
		
add_library(multi_objective_controller
  src/library/multi_objective_controller.cpp ${MATLAB_LIB})

target_link_libraries(lee_position_controller ${catkin_LIBRARIES})
add_dependencies(lee_position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})
add_dependencies(roll_pitch_yawrate_thrust_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(multi_objective_controller ${catkin_LIBRARIES})
add_dependencies(multi_objective_controller ${catkin_EXPORTED_TARGETS})

add_executable(lee_position_controller_node src/nodes/lee_position_controller_node.cpp)
add_dependencies(lee_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lee_position_controller_node
  lee_position_controller ${catkin_LIBRARIES})

add_executable(roll_pitch_yawrate_thrust_controller_node
  src/nodes/roll_pitch_yawrate_thrust_controller_node.cpp)
add_dependencies(roll_pitch_yawrate_thrust_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(roll_pitch_yawrate_thrust_controller_node
  roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})

add_executable(multi_objective_controller_node src/nodes/multi_objective_controller_node.cpp)
add_dependencies(multi_objective_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_objective_controller_node
  multi_objective_controller ${catkin_LIBRARIES})

install(TARGETS lee_position_controller roll_pitch_yawrate_thrust_controller multi_objective_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS lee_position_controller_node roll_pitch_yawrate_thrust_controller_node multi_objective_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
