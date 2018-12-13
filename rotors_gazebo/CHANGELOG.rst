^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.3 (2018-12-13)
------------------

2.2.2 (2018-12-12)
------------------

2.2.0 (2018-12-10)
------------------
* fixed xmlns:xacro link
* switch to xacro from xacro.py
* switch to package format 2
* Merge branch 'feature/mavlink_build' into feature/gazebo9-autobackport
* Merge pull request `#521 <https://github.com/ethz-asl/rotors_simulator/issues/521>`_ from scpeters/rotors_gazebo_run_depend
  rotors_gazebo/package.xml: alphabetize and add missing run depends
* rotors_gazebo: run_depend on joy, xacro
  Fixes `#501 <https://github.com/ethz-asl/rotors_simulator/issues/501>`_. Partial fix for `#500 <https://github.com/ethz-asl/rotors_simulator/issues/500>`_.
* rotors_gazebo/package.xml: alphabetize depends
* Merge pull request `#425 <https://github.com/ethz-asl/rotors_simulator/issues/425>`_ from ethz-asl/fix/add_ros_plugin_to_example_worlds
  fix/add_ros_plugin_to_example_worlds
* add ros_interface_plugin to all example worlds
* Merge pull request `#374 <https://github.com/ethz-asl/rotors_simulator/issues/374>`_ from ethz-asl/feature/wind_plugin_extended
  Extending wind plugin
* Merge pull request `#406 <https://github.com/ethz-asl/rotors_simulator/issues/406>`_ from ethz-asl/fix/waypoint_publisher
  FIX publish msg and not msg pointer in waypoint_publisher
* FIX publish msg in waypoint_publisher and waypoint_publisher_file `#373 <https://github.com/ethz-asl/rotors_simulator/issues/373>`_
* FIX waypoint_publisher; revert *msg -> msg and ros:shutdown() before exit
* FIX publish msg and not msg pointer in waypoint_publisher
* Merge pull request `#397 <https://github.com/ethz-asl/rotors_simulator/issues/397>`_ from ethz-asl/v2.1.1
  update to 2.1.1
* Additional requested changes for PR.
* Added example world and custom wind field text file, as well as documentation.
* Contributors: Fadri Furrer, Marius Fehr, Mina Kamel, Nicolas El Hayek, Steven Peters, Tim, Tim Taubner, Timo Hinzmann, michaelpantic

2.1.1 (2017-04-27)
-----------
* update maintainers
* Contributors: fmina

2.1.0 (2017-04-08)
-----------
* Remove iris model from warehouse world.
* Improve launch args for mav.launch and hovering example.
* Adding some explanation to the aero params YAML
* Modifying the Yosemite model to be aligned with the principal axis in x-y plane
* Some minor fixes to fw dynamics
* Implementing fixed-wing operation with a joystick
* Implementing the rest of parameter parsing and forces + moments calculations
* Adding max and min angle of attack as parameters and some minor renaming
* Making fixed-wing params more descriptive
* Implementing YAML parsing of fixed-wing aerodynamic parameters
* Parameterizing techpod mesh files and aerodynamic parameters
* Add position as optional arguments to hovering_example.
* iris.sdf is only built when Gazebo version is > 2.
* Removed un-needed parameters from basic.world.
* Corrected optical flow plugin name in a model.sdf file.
* Fixed LIDAR plugin name in SDF file (rotors prefix added).
* Turned the 'connect gazebo to ros' and 'connect ros to gazebo' topic names into global variables (couldn't work out how to make them global from the .world file, so this was the next best option).
* Converted gazebo_ros_interface_plugin from a model plugin to a world plugin.
* The ROS interface plugin is now attached to a static model in the world rather than being attahed to the firefly MAV.
* Added beginnings of Gazebo model for the purpose of inserting the ROS interface into the Gazebo world.
* Reverted back to commit 357ed0f254823e83e392e239a3ab7d32b595125e (Monday's commit just before .xacro files were merged).
* More attempted tweaks to fix errors with substitutaion_args/xacro.py. Added better debug print.
* Added debug messages to python files, moving .xacro files from rotors_gazebo/models/rotors_description/urdf into rotors_description/models/urdf.
* Changed variable in rotors_gazebo CMakeLists.txt to NO_ROS to match variable in rotors_gazebo_plugins.
* Added script files from sitl_gazebo (used to generate iris.sdf).
* Fixed bug where return statement was missing brackets.
* Added a ONLY_BUILD_IRIS_SDF variable to the rotors_gazebo CMakeLists.txt for PX4/Firmware support.
* Added all sitl_gazebo model files into rotors/gazebo/models/ directory.
* Added all the worlds from sitl_gazebo that were not present in rotors_simulator.
* Gazebo is now outputting debug messages to the console (verbose mode is turned on through launch file).
* Initial test change for px4_merge branch. Build and hovering example working.
* iris.sdf is only built when Gazebo version is > 2.
* Removed un-needed parameters from basic.world.
* Corrected optical flow plugin name in a model.sdf file.
* Fixed LIDAR plugin name in SDF file (rotors prefix added).
* Turned the 'connect gazebo to ros' and 'connect ros to gazebo' topic names into global variables (couldn't work out how to make them global from the .world file, so this was the next best option).
* Converted gazebo_ros_interface_plugin from a model plugin to a world plugin.
* The ROS interface plugin is now attached to a static model in the world rather than being attahed to the firefly MAV.
* Added beginnings of Gazebo model for the purpose of inserting the ROS interface into the Gazebo world.
* More attempted tweaks to fix errors with substitutaion_args/xacro.py. Added better debug print.
* Added debug messages to python files, moving .xacro files from rotors_gazebo/models/rotors_description/urdf into rotors_description/models/urdf.
* Changed variable in rotors_gazebo CMakeLists.txt to NO_ROS to match variable in rotors_gazebo_plugins.
* Added script files from sitl_gazebo (used to generate iris.sdf).
* Fixed bug where return statement was missing brackets.
* Added a ONLY_BUILD_IRIS_SDF variable to the rotors_gazebo CMakeLists.txt for PX4/Firmware support.
* Added all sitl_gazebo model files into rotors/gazebo/models/ directory.
* Added all the worlds from sitl_gazebo that were not present in rotors_simulator.
* change euroc to neo
* add neo11
* change euroc to neo9
* formatting
* add neo9 to rotors
* Merge branch 'master' into feature/px4_merge
  Pulling in changes from master.
* fixed indentation of grass.material
* Add SDF tag for octomapPubTopic and octomapServiceName and load the strings in the gazebo octomap plugin.
* Gazebo is now outputting debug messages to the console (verbose mode is turned on through launch file).
* Initial test change for px4_merge branch. Build and hovering example working.
* Extending Gazebo model path in keyboard interface launch file
* Adding interface for Python-uinput, a virtual keyboard joystick
* Adding yosemite model and world
* Change launch files to extend Gazebo model path instead of setting it
* Extending Gazebo resource path to include RotorS models
* Adding the [slightly modified] outdoor world from PX4 fork of RotorS
* Adding XML declaration to several existing world files
* Changing the fixed-wing orientation to be consistent with all other models
* Adding spherical_coordinates tags for geolocation in all the world files
* Adding separate launch files for fixed-wing models
* Removing magnetic_field tag from the world files
  Part of removing the use of gazebo's built-in magnetometer sensor in
  order to be compatible with Gazebo API version 5
* Adding magnetic field tag to all the world files
* Fixing all the RotorS launch files to work with more generic description files
* Adding 'wait_to_record_bag' parameter to all quad base description files
* Adding ability to start and stop rosbag recording on command
* Revert "Adding ability to start and stop rosbag recording on command"
  This reverts commit b747b360199629c31290b2892daa484dddb52d3c.
* Adding ability to start and stop rosbag recording on command
* Refactoring mavlink interface plugin more
* Adding vehicle and controller parameter files for new models
* Making the URDF description files more generic
* Adding 'world-name' as an argument to most launch files
* Commenting out a reference to an un-used and non-present package (?)
* added robot_state_publisher and joint_state_publisher in launch files
* added mavlink_interface flag to other iris launch files
* Updated to use mavros_msgs. Optionally add mavlink_interface to gazebo models
* waypoint publisher waits until subscriber is available
* style fix
* removed wrong vspace
* removed unnecessary mav_name_suffix and switched to using waypoint_publisher
  Added an optional delay parameter in the waypoint publisher to publish a
  waypoint at a later instance of time.
* Update and rename three_firefly_hovering_example.launch to firefly_swarm_hovering_example.launch
  add two more fireflys, and change "mav_suffix_name" to "mav_name_suffix"
* Update hovering_example.cpp
  add a waypoint for the hovering example
* Create three_firefly_hovering_example.launch
  add a example launch file with three fireflys
* Update spawn_mav.launch
  add a suffix to the arg "mav_name", in order to prevent namespace conflict when launching several mavs of the same type
* switch url website in package.xml to github repo
* add/change url and cleanup of package.xml files
  This addresses `#202 <https://github.com/ethz-asl/rotors_simulator/issues/202>`_.
* removed deprecated launch files
* remove _with_joy launch files, px4 Firmware start the node now in the correct namespace
* update iris with upstream changes
* update ardrone with upstream changes
* exposed common arguments to top level files
* update launch and xacro files to new names, update use of odometry plugin
* move px4 files to new directories
* Merge remote-tracking branch 'origin/master' into px4_nodes_upstreammerge
* Contributors: Andreas Antener, Fadri Furrer, Geoffrey Hunter, Haoyao Chen, Helen Oleynikova, James Goppert, Mina Kamel, Pavel, Thomas Gubler, acfloria, devbharat, fmina, pvechersky

2.0.1 (2015-08-10)
------------------
* fixed the bag plugin and the evaluation
* Contributors: Fadri Furrer

2.0.0 (2015-08-09)
------------------
* Change to use the new datatypes defined in mav_comm.
* Update hummingbird.yaml
  The original 0.68 did not involve the mass of rotors (0.009 each). Now change to 0.68 + 0.009*4
* Added subscriber for MultiDOFJointTrajectory messages and extended waypoint_publisher_file to test it. resolves `#243 <https://github.com/ethz-asl/rotors_simulator/issues/243>`_
* Contributors: Haoyao Chen, Helen Oleynikova, Markus Achtelik

1.1.6 (2015-06-11)
------------------

1.1.5 (2015-06-09)
------------------
* added install targets

1.1.4 (2015-05-28)
------------------

1.1.3 (2015-05-28)
------------------

1.1.2 (2015-05-27)
------------------

1.1.1 (2015-04-24)
------------------

1.1.0 (2015-04-24)
------------------
* initial Ubuntu package release
