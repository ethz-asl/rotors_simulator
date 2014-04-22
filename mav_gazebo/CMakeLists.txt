cmake_minimum_required(VERSION 2.8.3)
project(mav_gazebo)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs gazebo_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gazebo_pose_publisher src/gazebo_pose_publisher.cpp)
target_link_libraries(gazebo_pose_publisher ${catkin_LIBRARIES})
