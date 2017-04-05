cmake_minimum_required(VERSION 2.8.3)
project(rqt_rotors)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  rospy
  rotors_comm
  rqt_gui
  rqt_gui_py
  std_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS mavros_msgs rospy rotors_comm rqt_gui rqt_gui_py std_msgs std_srvs
  DEPENDS
)
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_rotors
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
