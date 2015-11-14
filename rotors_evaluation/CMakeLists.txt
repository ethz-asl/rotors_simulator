cmake_minimum_required(VERSION 2.8.3)
project(rotors_evaluation)

find_package(catkin REQUIRED COMPONENTS rospy)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS src/disturbance_eval.py
                               src/hovering_eval.py
                               src/waypoints_eval.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
