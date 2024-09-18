cmake_minimum_required(VERSION 3.8)
project(rotors_swift_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(swift_msgs REQUIRED)


#${mav_msgs_INCLUDE_DIRS} is the path for the header files in mav_msgs
include_directories(include ${mav_msgs_INCLUDE_DIRS})

add_executable(rotors_swift_interface src/swift_trans.cpp)
ament_target_dependencies(rotors_swift_interface rclcpp geometry_msgs mav_msgs swift_msgs)


install(TARGETS
  rotors_swift_interface
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


ament_package()
