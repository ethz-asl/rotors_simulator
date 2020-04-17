^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.3 (2018-12-13)
------------------

2.2.2 (2018-12-12)
------------------

2.2.0 (2018-12-10)
------------------
* fixed xmlns:xacro link
* switch to package format 2
* Merge pull request `#529 <https://github.com/ethz-asl/rotors_simulator/issues/529>`_ from ethz-asl/feature/gazebo9-noisydepth
  Feature/gazebo9 noisydepth
* Added parameters for noise models and reformatted code
* Cleanup of .xacro file
* Disabled sensor visual
* Fixed orientation and position of mesh relative to camera center (here, left imager) as per datasheet)
* Realsense definiton (positions etc wrong, to be corrected)
* Merge pull request `#464 <https://github.com/ethz-asl/rotors_simulator/issues/464>`_ from ethz-asl/fix/set_rate_back
  Set odometry rate back to 1/1 of real-time rate.
* Set odometry rate back to 1/1 of real-time rate.
* Merge pull request `#457 <https://github.com/ethz-asl/rotors_simulator/issues/457>`_ from ethz-asl/fix/odom_rate
  Set publish rate to 100 Hz instead of 1000Hz to better match real system
* Set publish rate to 100 Hz instead of 1000Hz to better match real system
* Fix/neo11 namespace (`#418 <https://github.com/ethz-asl/rotors_simulator/issues/418>`_)
  * Change argument from 'mav_name' to 'namespace' to ease spawning of multiple neo11s
* Merge pull request `#374 <https://github.com/ethz-asl/rotors_simulator/issues/374>`_ from ethz-asl/feature/wind_plugin_extended
  Extending wind plugin
* Requested changes for PR.
* Merge pull request `#397 <https://github.com/ethz-asl/rotors_simulator/issues/397>`_ from ethz-asl/v2.1.1
  update to 2.1.1
* Extended wind plugin to read custom wind values from a text file.
* Contributors: Andrea Tagliabue, Christian Witting, Fadri Furrer, Helen Oleynikova, Mina Kamel, Nicolas El Hayek, Timo Hinzmann, cwitting, michaelpantic, nelhayek

2.1.1 (2017-04-27)
-----------
* update maintainers
* Adding pressure plugin to urdf description
* Contributors: fmina, pvechersky

2.1.0 (2017-04-08)
-----------
* Implementing fixed-wing operation with a joystick
* Implementing the rest of parameter parsing and forces + moments calculations
* Parameterizing techpod mesh files and aerodynamic parameters
* Adding xacro parameter for wind speed mean
* The ROS interface plugin is now attached to a static model in the world rather than being attahed to the firefly MAV.
* Removed dead code from the .xacro macro for the ROS interface plugin.
* Removed dead commented code.
* More attempted tweaks to fix errors with substitutaion_args/xacro.py. Added better debug print.
* Odometry message now being captured by ROS interface plugin and published to ROS framework. Hovering example now works again.
* ROS interface plugin publishing actuators and link state messages to ROS platform
* ROS message interface plugin now converts magnetic field messages and publishes to ROS.
* Changed the name of the robot location GPS message to nav_sat_fix (since there is more than one GPS message type).
* Fixed issue with topic name differences between IMU/GPS plugins and the interface plugin.
* New Gazebo message interface plugin is loading correctly when hover sim is launched.
* Gazebo is now outputting debug messages to the console (verbose mode is turned on through launch file).
* The ROS interface plugin is now attached to a static model in the world rather than being attahed to the firefly MAV.
* Removed dead code from the .xacro macro for the ROS interface plugin.
* Removed dead commented code.
* More attempted tweaks to fix errors with substitutaion_args/xacro.py. Added better debug print.
* change euroc to neo
* Delete neo9_generic_odometry_sensor.gazebo
* Delete neo11_generic_odometry_sensor.gazebo
* change euroc_hex to neo11
* add neo11
* add neo9 to rotors
* Odometry message now being captured by ROS interface plugin and published to ROS framework. Hovering example now works again.
* ROS interface plugin publishing actuators and link state messages to ROS platform
* ROS message interface plugin now converts magnetic field messages and publishes to ROS.
* Changed the name of the robot location GPS message to nav_sat_fix (since there is more than one GPS message type).
* Fixed issue with topic name differences between IMU/GPS plugins and the interface plugin.
* New Gazebo message interface plugin is loading correctly when hover sim is launched.
* Gazebo is now outputting debug messages to the console (verbose mode is turned on through launch file).
* Launch and RVIZ config file for viewing Techpod model in RVIZ
* Adding meshes and URDF components for flaps in Techpod model
* Adding GPS to components snipets and mounting it on a fixed-wing Techpod model
* Fixing small details and removing references to gazebo plugins
* Adding Techpod URDF desciption files and a general URDF for fixed-wing
* Adding meshes for Techpod, extracted from actual CAD model
* Magnetometer refactoring to make use of constexpr and proper transform convention
* Modifying component_snipets to use the new custom magnetometer plugin
* Adding some noise based on ADIS16448 specs
* Adding a macro for magnetometer component
* Fixing all the RotorS launch files to work with more generic description files
* Adding 'wait_to_record_bag' parameter to all quad base description files
* Adding ability to start and stop rosbag recording on command
* Fixing a couple of small bugs in mavlink interface plugin
* Refactoring mavlink interface plugin more
* Adding vehicle and controller parameter files for new models
* Removing unfinished parts of px4 dummy controller and gazebo mavlink interface. Refactoring gazebo mavlink interface.
* Removing the unfinished wing plugin and the VTOL model
* Making the URDF description files more generic
* Adding 'world-name' as an argument to most launch files
* added optical link also to stereo camera
* add optical joint for camera
* fix namespace variable
  missing $ sign
* Updated to use mavros_msgs. Optionally add mavlink_interface to gazebo models
* started cleanup of odometry plugin
* removed unnecessary mav_name_suffix and switched to using waypoint_publisher
  Added an optional delay parameter in the waypoint publisher to publish a
  waypoint at a later instance of time.
* Update hummingbird.xacro
  use "mav_name" arg which has an additional suffix, in order to prevent namespace conflict when launching several mavs of the same type.
* Update pelican.xacro
  use "mav_name" arg which has an additional suffix, in order to prevent namespace conflict when launching several mavs of the same type.
* Update firefly.xacro
  use "mav_name" arg which has an additional suffix, in order to prevent namespace conflict when launching several mavs of the same type.
* switch url website in package.xml to github repo
* Added the mavlink interface plugin to the iris model via sdf file
* replace middle dot with a period
* update iris with upstream changes
* update ardrone with upstream changes
* Merge remote-tracking branch 'upstream/master' into px4_nodes_upstreammerge3
* remove old/outdated urdf/xacro files
* update launch and xacro files to new names, update use of odometry plugin
* move px4 files to new directories
* Contributors: Andre Phu-Van Nguyen, Fadri Furrer, Geoffrey Hunter, Haoyao Chen, Michael Burri, Mina Kamel, Pavel, Raghav Khanna, Thomas Gubler, devbharat, fmina, pvechersky

2.0.1 (2015-08-10)
------------------
* fixed the bag plugin and the evaluation
* Contributors: Fadri Furrer

2.0.0 (2015-08-09)
------------------
* fixed base_link issue with gazebo 2.2
* added a mesh for the vi_camera and fixed the mesh for the vi_sensor
* added possibility to add own meshes of propellers
* updated gazebo links to also contain the namespace
* Contributors: Fadri Furrer, Helen Oleynikova, Michael Burri

1.1.6 (2015-06-11)
------------------

1.1.5 (2015-06-09)
------------------

1.1.4 (2015-05-28)
------------------

1.1.3 (2015-05-28)
------------------

1.1.2 (2015-05-27)
------------------
* added max depth cam range as param
* pointCloudCutoffMax tag added for depth cam

1.1.1 (2015-04-24)
------------------

1.1.0 (2015-04-24)
------------------
* initial Ubuntu package release
