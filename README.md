euroc_simulator
===============

This is a new branch of the euroc_simulator which intends to use library code from the PX4 repo
to simulate (until now) multicopters. The idea is to import as much code and structure from the PX4 project as possible
in order to avoid divergence and in order to benefit from updates.

What you need to do to make it work
-----------------------------------

Note: There is still alot of work in progress so don't expect anything to work yet!

1. Create a catkin workspace http://wiki.ros.org/catkin/Tutorials/create_a_workspace
2. Clone this repository into the src folder inside your newly generated workspace
3. Clone the PX4 repository to some destination and change branch to 'ROS_shared_lib_base_class'
  (I will push this branch very soon)
4. Open the file "euroc_simulator/mav_control/CMakelist.txt" and modify the follwing line to match your situation:
    "set(PX4_DIR local_PX4_repo_path)" <br />
    "local_PX4_repo_path" should match the path where you cloned the PX4 repo into before
5. In order to make things work, I had to add the following ROS packages to my catkin workspace: <br />
  mav_comm, octomap_msgs, octomap_ros
6. Inside your catkin workspace type "catkin_make" to build your packages.
7. In case it really worked type e.g: <br />
  "roslaunch mav_gazebo vtol_empty_world_with_joy.launch" which should start the simulation (you will need a joystick)
    

