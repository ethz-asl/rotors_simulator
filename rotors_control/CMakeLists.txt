cmake_minimum_required(VERSION 2.8.3)
project(rotors_control)

add_definitions(-std=c++11)


#set(MPC_POSITION_CONTROLLER_SOLVER "CVXGEN")
set(MPC_POSITION_CONTROLLER_SOLVER "FORCES")

if(MPC_POSITION_CONTROLLER_SOLVER STREQUAL "CVXGEN")
 add_library(cvxgen_solver 
  src/library/CVXGEN_Solver/ldl.c 
  src/library/CVXGEN_Solver/solver.c 
  src/library/CVXGEN_Solver/matrix_support.c 
  src/library/CVXGEN_Solver/util.c 
 )
 
  add_definitions(-DUseCVXGENSolver)
endif()

if(MPC_POSITION_CONTROLLER_SOLVER STREQUAL "FORCES")
 add_library(forces_solver
  src/library/FireFlyOffsetFreeMPC.c
 )
 
 set(CMAKE_BUILD_TYPE Release) 	#otherwise it is slow
 add_definitions(-DUseForcesSolver)
endif()
 
 
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  roscpp
  sensor_msgs
  cmake_modules
  message_generation
)

find_package(Eigen REQUIRED)

add_service_files(
  FILES
  InitKF.srv
  calibrateKF.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES lee_position_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs roscpp sensor_msgs
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

add_library(MPC_position_controller
	src/library/MPC_position_controller.cpp
	src/library/SteadyStateCalculation.cpp
    src/library/KFDisturbanceObserver.cpp)
    

add_library(PID_attitude_controller
  src/library/PID_attitude_controller.cpp
)

target_link_libraries(lee_position_controller ${catkin_LIBRARIES})
add_dependencies(lee_position_controller ${catkin_EXPORTED_TARGETS})

if(MPC_POSITION_CONTROLLER_SOLVER STREQUAL "FORCES")
  target_link_libraries(MPC_position_controller forces_solver ${catkin_LIBRARIES})
endif()

if(MPC_POSITION_CONTROLLER_SOLVER STREQUAL "CVXGEN")
  target_link_libraries(MPC_position_controller cvxgen_solver ${catkin_LIBRARIES})
endif()

add_dependencies(MPC_position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(PID_attitude_controller ${catkin_LIBRARIES})
add_dependencies(PID_attitude_controller ${catkin_EXPORTED_TARGETS})

add_executable(lee_position_controller_node src/nodes/lee_position_controller_node.cpp)
add_dependencies(lee_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lee_position_controller_node lee_position_controller ${catkin_LIBRARIES})

add_executable(MPC_position_controller_node src/nodes/MPC_position_controller_node.cpp)
add_dependencies(MPC_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(MPC_position_controller_node MPC_position_controller ${catkin_LIBRARIES})

add_executable(PID_attitude_controller_node src/nodes/PID_attitude_controller_node.cpp)
add_dependencies(PID_attitude_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(PID_attitude_controller_node PID_attitude_controller ${catkin_LIBRARIES})
