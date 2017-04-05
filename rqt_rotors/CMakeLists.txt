cmake_minimum_required(VERSION 2.8.3)
project(rqt_rotors)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS mavros_msgs rospy rqt_gui rqt_gui_py
  DEPENDS
)
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/hil_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
