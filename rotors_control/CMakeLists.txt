cmake_minimum_required(VERSION 3.8)
project(rotors_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
 
find_package(Eigen3 REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)


include_directories(
  include
  ${rotors_control_INCLUDE_DIRS}
  ${mav_msgs_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_INCLUDE_DIRS}
)

add_library(roll_pitch_yawrate_thrust_controller src/library/roll_pitch_yawrate_thrust_controller.cpp)

add_executable(roll_pitch_yawrate_thrust_controller_node
src/nodes/roll_pitch_yawrate_thrust_controller_node.cpp)

ament_target_dependencies(roll_pitch_yawrate_thrust_controller_node 
rclcpp
geometry_msgs
Eigen3
mav_msgs
nav_msgs
geometry_msgs
std_msgs
actuator_msgs
)

target_link_libraries(roll_pitch_yawrate_thrust_controller_node roll_pitch_yawrate_thrust_controller)

install(TARGETS
  roll_pitch_yawrate_thrust_controller
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS roll_pitch_yawrate_thrust_controller_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include) 
ament_export_libraries(
  roll_pitch_yawrate_thrust_controller
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
