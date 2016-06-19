cmake_minimum_required(VERSION 2.8.3)
project(rotors_comm)
find_package(catkin REQUIRED
  COMPONENTS cmake_modules geometry_msgs message_generation octomap_msgs std_msgs)

# Declare the service files to be built
add_service_files(
  FILES
  Octomap.srv
  RecordRosbag.srv
)

add_message_files(
  FILES
  WindSpeed.msg
)

generate_messages(DEPENDENCIES geometry_msgs octomap_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
