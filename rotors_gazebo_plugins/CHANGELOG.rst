^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.3 (2018-12-13)
------------------
* Added dependencies to gazebo_plugins
* Contributors: michaelpantic

2.2.2 (2018-12-12)
------------------
* Fixes issues with build on buildfarm and old ubuntu systems (added MAVROS dependencies, version checking, protobuf-dev etc)
* Contributors: michaelpantic

2.2.0 (2018-12-10)
------------------
* Major changes in CMakeLists and package definition for Gazebo version compatibility
* Code updated to Gazebo9 API
* Automatic backport script added for pre-9 compatibility
* Added NoisyDepth Plugin
* Contributors: Fadri Furrer, Helen Oleynikova, Michael Pantic, Mina Kamel, Nicolas El Hayek, Timo Hinzmann, Zachary Taylor, fmina, kajabo, michaelpantic, nelhayek, orangelynx, ptrkfry

2.1.1 (2017-04-27)
-----------
* Merge pull request `#377 <https://github.com/ethz-asl/rotors_simulator/issues/377>`_ from ethz-asl/feature/pressure-plugin
  Feature/pressure-plugin
* More verbose variable names in pressure calculations.
* update maintainers
* Giving more verbose names to the constants
* Adapting the pressure plugin to work with Gazebo transport interface
* Adding Gazebo interface for fluid pressure message
* Initial commit for pressure plugin.
* Contributors: Timo Hinzmann, fmina, pvechersky

2.1.0 (2017-04-08)
-----------
* fixes issue 371
* Correct gazebo versioning
* Adding yaml_cpp_catkin, along with catkin_simple, as a dependency for ROS configuration
* Adding find_library to FindYamlCpp and linking to it
* Adding FindYamlCpp cmake module to locate yaml-cpp include dirs
* Making some vectors, YAML nodes, and long calculation variables as const
* Minor text fixes.
* Minor fixes
* Some minor fixes to fw dynamics
* Implementing fixed-wing operation with a joystick
* Fixing the TwistStamped message publishing
* Add conversion of RollPitchYawrateThrust msgs from ROS to Gazebo to allow Joystick node input in Gazebo plugins
* Implementing the rest of parameter parsing and forces + moments calculations
* Implementing handling of actuator command to fixed-wing dynamics plugin
* Adding max and min angle of attack as parameters and some minor renaming
* Making Gazebo Actuators msg same as mav_msgs Actuators msg
* Making fixed-wing params more descriptive
* Implementing YAML parsing of fixed-wing aerodynamic parameters
* Parameterizing techpod mesh files and aerodynamic parameters
* First outline of the fixed-wing dynamics plugin
* Adding wind speed recording to rosbag plugin.
* Adapting Rotors plugin to use the new external force and wind speed mav_comm topics
* Adapting wind plugin to publish the WIND SPEED message
* Adding ability to convert Gazebo wind speed msgs into ROS wind speed msgs
* Adding wind speed publishing to the wind plugin
* Fixing a message type typo in the comment
* Merge pull request `#343 <https://github.com/ethz-asl/rotors_simulator/issues/343>`_ from acfloria/features/octomap_plugin_gazebo_origin
  Octomap Plugin Gazebo Origin
* run clang-format through all our plugins
* fix style and removed some comments
* Removed debug code, tidied up comments.
* Added helper methods for GZ to ROS and ROS to GZ header conversion.
* Added collision check for map which stored ConnectHelperStorage items by Gazebo topic name.
* ROS interface plugin no longer checks and uses the namespace variable from SDF file.
* Fixed code style of constants in MAVLink interface .cpp file.
* Added todo 'add noise' to IMU plugin.
* GPS plugin now uses default message topics from mav_msgs.
* Fixed up TwistStamped message type, and moved from a sensor to geomtery message namespace.
* Motor model now uses default message types defined in mav_msgs. Made types more descriptive.
* Re-enabled used of default topics from mav_msgs package in IMU plugin.
* Tidied up code style in MAVlink interface plugin.
* Removed unused and duplicate MotorSpeed.proto message type.
* Moved the gimbal controller, GST camera, LIDAR and optical flow plugins to 'external' directories.
* Moved Geotagged images plugin code files into 'external' directories.
* Added default values for passed-in variables to rotors_gazebo_plugins CMake file.
* Use ignition::math::Vector3d instead of gazebo::math::Vector3 for Gazebo version 6 or newer in the octomap plugin.
* Explicitly added specific MAVLink header for debugging.
* Fixed Gazebo topic namespaces into the MAVlink interface plugin.
* All Gazebo plugins now use topics on the default Gazebo namespace.
* ROS interface plugin now uses /gazebo/rotors/ namespace for the two 'connect' topics.
* Fixed up protobuf message numbering.
* Gazebo namespace is now required for all ROS to Gazebo messages. All plugins should be now setting this correctly.
* All ROS to Gazebo message types now support a Gazebo namespace setting.
* ACTUATORS message type now uses Gazebo namespace to create ROS to Gazebo connection.
* Gazebo namespace is now required for the connect gazebo to ros messages.
* Magnetometer, motor model, multirotor base and wind plugins now set Gazebo namespace.
* Controller interface, GPS and IMU plugins now set the Gazebo namespace value in the ROS interface message.
* All topics in odometry plugin now configure Gazebo namespace for ROS interface plugin.
* Gazebo namespace variable is now used in ROS interface plugin if provided.
* Added support into GazeboRosInterfacePlugin to use a provided Gazebo namespace.
* Added Gazebo namespace string to 'connect to ROS' messages.
* Revert "Added mavlink header files (for mavlink plugin)."
  This reverts commit 0898d181476b8646b3fd1dfad9e229299be4dbd7.
* Dropped prefix to MAVLink header include.
* Fixed up MAVLink header check in CMakeFile.txt.
* Added ability for CMakeFiles.txt to search for MAVLink header files, unless provided by command-line variable.
* Tidied up header inclusions.
* Revert "Added mavlink header files (for mavlink plugin)."
  This reverts commit 285468c184486cdd1bb2b48268b24088d7155a4f.
* Added mavlink header files (for mavlink plugin).
  (cherry picked from commit 285468c184486cdd1bb2b48268b24088d7155a4f)
* fix linking on os x
* Fixed up if statement in CMakeLists.txt.
* Added CMake check for Gazebo >= v5.0 if building optical flow plugin.
* Tidied up CMakeLists.txt by creating list that collects targets to be installed rather than manually specifying them at the end.
* Removed system.hh include from geotagged images plugin for Gazebo 5 support.
* Revert "Added namespace functionality to the ROS interface plugin. The 'connect' messages now get passed a Gazebo namespace as well as a topic name."
  This reverts commit 30ec6fea1e404badf5df6c5c84f31dbb63f12155.
* Added namespace functionality to the ROS interface plugin. The 'connect' messages now get passed a Gazebo namespace as well as a topic name.
* Removed duplicated protobuf message types in favour of using Gazebo ones instead.
* Fixed header inclusion naming error.
* Tidied up code, removed dead, commented-out ROS references.
* Renamed protobuf  namespace to , closes `#339 <https://github.com/ethz-asl/rotors_simulator/issues/339>`_.
* Renamed 'SensorImu.proto' to 'Imu.proto', closes `#341 <https://github.com/ethz-asl/rotors_simulator/issues/341>`_.
* Added commeneted out debug print code for MAVLink GPS message.
* Added debug print statements to MAVLink interface plugin (commented out for now).
* The IMU orientation quaternion is now done the 'PX4' way.
* Fixed typo in MAVLink interface plugin where usec=nsec*1000, replaced with usec=nsec/1000.
* Commented out debug prints in MAVLink interface plugin.
* Removed debug print for motor velocities.
* Added debug printing to MAVLink msg callbacks.
* Added missing semicolon.
* Added better motor ref. velocity debug printing.
* Added more debug printing to MAVLink interface plugin.
* Added todo for absolute gps_position topic (used by MAVlink interface and geotagged image plugins).
* Added debug printing to the MAVLink interface plugin.
* Added status messages to CMakeLists.txt which prints input parameter info.
* Updated call to calcFlow() in gazebo_optical_flow_plugin to match API changes made in OpticalFlow repo.
* Re-added transform broadcaster support to odometry plugin (through ROS interface plugin). Issue `#332 <https://github.com/ethz-asl/rotors_simulator/issues/332>`_.
* Removed duplicate initialisation of topic name in gazebo_motor_model.h.
* Re-added default topic names from mav_msgs dependency back into all plugin header files. Tidied up code comments.
* Re-enabled default msg topics from mav_msgs in the Odometry plugin.
* Fixed typo in CMakeLists.txt when including ADDITIONAL_INCLUDE_DIRS.
* Changed the passed in CMake variable from INCLUDE_DIRECTORIES to ADDITIONAL_INCLUDE_DIRS.
* Added single dependency to mav_comm into Odometry plugin (as test). Printing INCLUDE_DIRECTORIES variable from CMakeLists.txt.
* Standarised commenting in ROS interface plugin and moved method descriptions into the header file.
* Added missing include of 'common.h' in the optical flow plugin.
* Tidied up commenting in ROS interface plugin.
* Removed debug msg print from end of Load() method.
* Removed dead comments and fixed up frame ID for the GPS message.
* Tidied up comments in 'common.h'.
* Added missing commmon.h includes to two .cpp plugin files.
* Added debug print switch for the plugin's Load() method. Added debug print switch comments.
* Removed debug print 'Gazebo node created at...'.
* Added external library liftdrag_plugin.cpp/.h.
* Updated variable in CMakeLists.txt to BUILD_MAVLINK_INTERFACE_PLUGIN.
* Tweaked BUILD_MAVLINK_PLUGIN variable to follow convention of other boolean flags in CMakeLists.txt.
* Added boolean constants to enable/disable debug printing.
* Silenced debug print in wind plugin.
* Fixed typo in setting forces in the wrench message in the wind plugin.
* Removed dead commented-out code from CMakeLists.txt.
* Replaced manual list of .proto files in CMakeLists.txt with file(GLOB ...) syntax.
* Added SHARED qualifer to Gazebo plugin libraries in CMakeLists.txt.
* Removed dead ROS message include from Magnetometer plugin.
* Removed dead ROS message header include from IMU plugin.
* Removed tf::transform related variables from Odometry plugin header.
* Removed ROS ros::Duration() calls from the odometry plugin.
* Removed dead ROS includes from the odometry plugin.
* Removed ROS dependency from the Gazebo GPS plugin, tweaked the NavSatFix protobuf message to accomodate this.
* Implemented GzTransformStampedMsgCallback().
* Implemented GzPositionStampedMsgCallback().
* Implemented GzPoseWithCovarianceStampedMsgCallback().
* Fixed seg fault, needed to resize part of ROS message in the joint states callback.
* Implemented GzJointStateMsgCallback(), but now seg fault is occurring (due to something in the last two commits?).
* Removed ROS dependencies from the Gazebo wind plugin.
* Octomap plugin is now built on a conditional basis, silenced debug printing from the ROS interface plugin during runtime.
* Tidied up debug printing and comments.
* Replaced include_directories() call with target_include_directories().
* Added Boost as a dependancy of the optical flow plugin.
* Added debug info.
* Linked OpitcalFlow library to Gazebo plugin.
* Fixed bug in checking for header file include variable, and return if variable not found.
* Added check for OpticalFlow_INCLUDE_DIRS.
* Readded write to CMAKE_MODULE_PATH, but now appends rather than overwrites.
* Stopped CMAKE_MODULE_PATH being set to ./cmake in rotors_gazebo_plugins.
* Added debug printing to CMakeLists.txt.
* Added debug printing to CMakeLists.txt.
* Added debug printing to CMakeLists.txt.
* Printing out CMAKE_MODULE_PATH during build.
* CMakeLists.txt now used find_package(OpticalFlow).
* Gazebo optical flow plugin is now built as a shared library.
* Added  when optical flow submodule is build.
* Added the OpticalFlow/include directory via include_directories() command in CMakeLists.txt.
* Fixed error in path to OpticalFlow submodule.
* Fixed naming issue with CMAKE_CURRENT_SOURCE_DIR.
* Optical flow subdirectory command is now passed a path depending on CMAKE_CURRENT_SOURCE_DIRECTORY.
* Fixed path for optical flow plugin in add_subdirectory() command.
* Updated path to OpticalFlow 'subdirectory' in CMakeLists.txt.
* Added CMake code to build PX4 optical code module (experimental).
* Added CMake code for optical flow plugin, and is now only built if a cmake command-line argument is set to true.
* LIDAR plugin is now built as a shared library.
* Added CMake code for LIDAR plugin.
* Added CMake code for geotagged images plugin.
* Added remaining plugins from sitl_gazebo repo (.cpp and .h files). Have not updated CMake files yet.
* Turned the 'connect gazebo to ros' and 'connect ros to gazebo' topic names into global variables (couldn't work out how to make them global from the .world file, so this was the next best option).
* Converted gazebo_ros_interface_plugin from a model plugin to a world plugin.
* The ROS interface plugin is now attached to a static model in the world rather than being attahed to the firefly MAV.
* Added beginnings of Gazebo model for the purpose of inserting the ROS interface into the Gazebo world.
* Removed unused variables and dead comments.
* Fixed Gazebo topic name for joint_states.
* Improved the debug printing statements in .cpp files.
* Fixed topic names used in gazebo_odometry_plugin.cpp.
* Fixed typo.
* Added check to GazeboRosInterfacePlugin to make sure publisher doesn't already exist.
* Refactored arrangement of function definitions.
* Fixed include in geo_mag_declination.cpp.
* Added missing geo_mag_declination.cpp file.
* Fixed Gazebo topic name for the Gazebo plugin.
* Reverted back to commit 357ed0f254823e83e392e239a3ab7d32b595125e (Monday's commit just before .xacro files were merged).
* Added debug messages to python files, moving .xacro files from rotors_gazebo/models/rotors_description/urdf into rotors_description/models/urdf.
* Added more debug info.
* Updated debug printing.
* Updated debug printing info in GazeboMotorModel.
* Fixed up topic paths in GazeboMotorModel.
* Printing out the motor velocity topic path.
* Changed Gazebo topic name for the actual motor speed topics.
* Added print message when GazeboMavlinkInterface gets loaded.
* Specifically added the protobuf message library as a dependency to the MAVlink plugin.
* Removed SHARED qualifier for gazebo_mavlink_interface in the cmake file.
* Added geo_mag_declination from sitl_gazebo repo.
* Added mavlink header files (for mavlink plugin).
* Added Lidar, MotorSpeed and OpticalFlow messages from sitl_gazebo.
* Fixed include.
* Adjusted CMakeLists.txt for mavlink plugin.
* Changed behaviour of MAVLINK_INTERFACE variable in CMakeLists so PX4 firmware can set it to TRUE.
* Replaced gazebo mavlink interface plugin with version from sitl_gazebo.
* Changed CMakeList variable name that controls ROS dependency inclusion to 'NO_ROS'. CMakeLists.txt now checks to see if it's defined outside of it's own code (designed to be set via command-line argument).
* Modified CMakeLists whitespace formatting.
* The ROS interface plugin is no longer built if ROS_DEPENDENCY = FALSE. cmake/make builds o.k. with no ROS dependencies!
* Removed unused ROS dependency includes from odometry plugin.
* Removed un-used headers from gazebo_motor_model.h.
* Added commented code from sitl_gazebo r.e. modelling the change in propulsion on rotor due to relative air velocity. Added comments about why this code is not active.
* Finished removing ROS dependencies from GazeboMotorModel.
* Added new protobuf messages for GazeboMotorModel plugin. Half-way through removing ROS depdencies in GazeboMotorModel.
* Removing dependencies on mav_msgs package (for ease of testing purposes).
* Removed unused header include (was a ROS dependency).
* Pubs and subs are now created on first call to OnUpdate() so to be sure the ROS interface plugin has been loaded.
* Hovering example now working with new way of setting up the ROS interface plguin.
* Basic functionality of ROS->Gazebo message converter working.
* Adding ROS to Gazebo message conversion functionality.
* Refactoring in preparation for adding from ROS to Gazebo message conversion support.
* Converting GazeboControllerInterface so to have no ROS dependencies.
* Reverted GazeboControllerInterface just to test new non-singleton way of connecting messages using the ROS interface. Hovering example works.
* Removed references to singleton pattern for ROS interface plugin.
* Removed static .getInstance method to register gazebo to ROS connections, now using another message type instead.
* GazeboBagPlugin is only built if ROS is present.
* Fixed COPY function call.
* Add cmake module for finding Eigen package. Adjusting CMakeLists to build without ROS.
* Fixed paste error in .cpp file.
* Added Transform related messages and transformation publishing from the Odometry plugin.
* Fixed bug when building a new msg by pointing to parts of existing msgs, by using CopyFrom() instead.
* Added comments to ConnectToRos() helper classes.
* All converted plugins now use ConnectToRos().
* ConnectToRos() now working for multiple message types.
* Refactored method names associated with connecting Gazebo topics to ROS topics.
* Odometry messages are now being correctly published to ROS framework via AttachTo() function.
* Gazebo subscriber callback is now being called via AttachTo() function. AttachHelper() is now a member funciton.
* Commit before making AttachHelper a member function.
* Started adding a generic AttachTo() method for the ROS interface plugin. Compiling but not yet linking basic odometry message yet.
* Odometry message now being captured by ROS interface plugin and published to ROS framework. Hovering example now works again.
* Started modifying GazeboOdometryPlugin to publish Gazebo messages and removing the ROS dependencies.
* ROS interface plugin publishing actuators and link state messages to ROS platform
* Working on the conversion of Gazebo Actuators and JointState messages into ROS messages.
* GazeboMultirotorBase is now publishing Gazebo messages. Repeated Header type in protobuf messages has been extracted and is now shared between other message types, closes `#326 <https://github.com/ethz-asl/rotors_simulator/issues/326>`_. Added debug print to plugin Load() methods to see what plugins are been run by hovering example.
* Renamed gazebo_msg_interface_plugin to gazebo_ros_interface_plugin, closes `#324 <https://github.com/ethz-asl/rotors_simulator/issues/324>`_.
* ROS message interface plugin now converts magnetic field messages and publishes to ROS.
* Added protobuf message type for magnetometer sensor. Magnetometer plugin now publishes Gazebo messages.
* Changed the name of the robot location GPS message to nav_sat_fix (since there is more than one GPS message type).
* Fixed issue with topic name differences between IMU/GPS plugins and the interface plugin.
* Converted ROS asserts to Gazebo asserts, removed ROS header file inclusions from IMU files.
* Adjusting the topic names and removing duplicate model names from namespace.
* Added TwistedStamp protobuf message type for sending ground position messages within Gazebo.
* Renamed GPS message. Gazebo GPS plugin should now emit NavSatFix messages on the Gazebo framework.
* Added GPS protobuf message type.
* Modifying Gazebo GPS plugin to publish Gazebo messages instead of ROS msgs. Commit before adding GPS protobuf message.
* All fields from Gazebo IMU msg copied into ROS IMU msg.
* Working on gazebo to ROS interface plugin. Fixed bug with IMU message header types.
* New Gazebo message interface plugin is loading correctly when hover sim is launched.
* Gazebo is now outputting debug messages to the console (verbose mode is turned on through launch file).
* Fixed error where Google protobuf message indexes where outside limits.
* Added template class for new Gazebo plugin to act as message interface to both Mavlink and ROS. Code is just a template, no functionality yet implemented.
* Fixed bug with un-resolved symbol. Hovering sim now works fine, although it shouldn't be getting any IMU data anymore.
* IMU plugin is now compiling. Crashing on hover sim start due to undefined symbol.
* Removed un-used extra config variable from cmake file. IMU message type is now a custom type.
* Compiled protobuf files are now being copied into devel space, and can be included from other C++ files.
* Begun reworking IMU plugin to publish Gazebo messages. Protobuf files being built/included using CMakeLists.txt.
* Added method/class comments.
* Added namespace comment as per Google style guide.
* fix linking on os x
* Fixed up if statement in CMakeLists.txt.
* Added CMake check for Gazebo >= v5.0 if building optical flow plugin.
* Tidied up CMakeLists.txt by creating list that collects targets to be installed rather than manually specifying them at the end.
* Removed system.hh include from geotagged images plugin for Gazebo 5 support.
* Revert "Added namespace functionality to the ROS interface plugin. The 'connect' messages now get passed a Gazebo namespace as well as a topic name."
  This reverts commit 30ec6fea1e404badf5df6c5c84f31dbb63f12155.
* Added namespace functionality to the ROS interface plugin. The 'connect' messages now get passed a Gazebo namespace as well as a topic name.
* Removed duplicated protobuf message types in favour of using Gazebo ones instead.
* Return the origin of the gazebo coordinates in lat/long/alt as part of the octomap service response.
* Fixed header inclusion naming error.
* Tidied up code, removed dead, commented-out ROS references.
* Renamed protobuf  namespace to , closes `#339 <https://github.com/ethz-asl/rotors_simulator/issues/339>`_.
* Renamed 'SensorImu.proto' to 'Imu.proto', closes `#341 <https://github.com/ethz-asl/rotors_simulator/issues/341>`_.
* Added commeneted out debug print code for MAVLink GPS message.
* Added debug print statements to MAVLink interface plugin (commented out for now).
* The IMU orientation quaternion is now done the 'PX4' way.
* Fixed typo in MAVLink interface plugin where usec=nsec*1000, replaced with usec=nsec/1000.
* Commented out debug prints in MAVLink interface plugin.
* Removed debug print for motor velocities.
* Added debug printing to MAVLink msg callbacks.
* Added missing semicolon.
* Added better motor ref. velocity debug printing.
* Added more debug printing to MAVLink interface plugin.
* Added todo for absolute gps_position topic (used by MAVlink interface and geotagged image plugins).
* Added debug printing to the MAVLink interface plugin.
* Added status messages to CMakeLists.txt which prints input parameter info.
* Updated call to calcFlow() in gazebo_optical_flow_plugin to match API changes made in OpticalFlow repo.
* Re-added transform broadcaster support to odometry plugin (through ROS interface plugin). Issue `#332 <https://github.com/ethz-asl/rotors_simulator/issues/332>`_.
* Removed duplicate initialisation of topic name in gazebo_motor_model.h.
* Re-added default topic names from mav_msgs dependency back into all plugin header files. Tidied up code comments.
* Re-enabled default msg topics from mav_msgs in the Odometry plugin.
* Fixed typo in CMakeLists.txt when including ADDITIONAL_INCLUDE_DIRS.
* Changed the passed in CMake variable from INCLUDE_DIRECTORIES to ADDITIONAL_INCLUDE_DIRS.
* Added single dependency to mav_comm into Odometry plugin (as test). Printing INCLUDE_DIRECTORIES variable from CMakeLists.txt.
* Standarised commenting in ROS interface plugin and moved method descriptions into the header file.
* Added missing include of 'common.h' in the optical flow plugin.
* Tidied up commenting in ROS interface plugin.
* Removed debug msg print from end of Load() method.
* Removed dead comments and fixed up frame ID for the GPS message.
* Tidied up comments in 'common.h'.
* Added missing commmon.h includes to two .cpp plugin files.
* Added debug print switch for the plugin's Load() method. Added debug print switch comments.
* Removed debug print 'Gazebo node created at...'.
* Added external library liftdrag_plugin.cpp/.h.
* Updated variable in CMakeLists.txt to BUILD_MAVLINK_INTERFACE_PLUGIN.
* Tweaked BUILD_MAVLINK_PLUGIN variable to follow convention of other boolean flags in CMakeLists.txt.
* Added boolean constants to enable/disable debug printing.
* Silenced debug print in wind plugin.
* Fixed typo in setting forces in the wrench message in the wind plugin.
* Removed dead commented-out code from CMakeLists.txt.
* Replaced manual list of .proto files in CMakeLists.txt with file(GLOB ...) syntax.
* Added SHARED qualifer to Gazebo plugin libraries in CMakeLists.txt.
* Removed dead ROS message include from Magnetometer plugin.
* Removed dead ROS message header include from IMU plugin.
* Removed tf::transform related variables from Odometry plugin header.
* Removed ROS ros::Duration() calls from the odometry plugin.
* Removed dead ROS includes from the odometry plugin.
* Removed ROS dependency from the Gazebo GPS plugin, tweaked the NavSatFix protobuf message to accomodate this.
* Implemented GzTransformStampedMsgCallback().
* Implemented GzPositionStampedMsgCallback().
* Implemented GzPoseWithCovarianceStampedMsgCallback().
* Fixed seg fault, needed to resize part of ROS message in the joint states callback.
* Implemented GzJointStateMsgCallback(), but now seg fault is occurring (due to something in the last two commits?).
* Removed ROS dependencies from the Gazebo wind plugin.
* Octomap plugin is now built on a conditional basis, silenced debug printing from the ROS interface plugin during runtime.
* Tidied up debug printing and comments.
* Replaced include_directories() call with target_include_directories().
* Added Boost as a dependancy of the optical flow plugin.
* Added debug info.
* Linked OpitcalFlow library to Gazebo plugin.
* Fixed bug in checking for header file include variable, and return if variable not found.
* Added check for OpticalFlow_INCLUDE_DIRS.
* Readded write to CMAKE_MODULE_PATH, but now appends rather than overwrites.
* Stopped CMAKE_MODULE_PATH being set to ./cmake in rotors_gazebo_plugins.
* Added debug printing to CMakeLists.txt.
* Added debug printing to CMakeLists.txt.
* Added debug printing to CMakeLists.txt.
* Printing out CMAKE_MODULE_PATH during build.
* CMakeLists.txt now used find_package(OpticalFlow).
* Gazebo optical flow plugin is now built as a shared library.
* Added  when optical flow submodule is build.
* Added the OpticalFlow/include directory via include_directories() command in CMakeLists.txt.
* Fixed error in path to OpticalFlow submodule.
* Fixed naming issue with CMAKE_CURRENT_SOURCE_DIR.
* Optical flow subdirectory command is now passed a path depending on CMAKE_CURRENT_SOURCE_DIRECTORY.
* Fixed path for optical flow plugin in add_subdirectory() command.
* Updated path to OpticalFlow 'subdirectory' in CMakeLists.txt.
* Added CMake code to build PX4 optical code module (experimental).
* Added CMake code for optical flow plugin, and is now only built if a cmake command-line argument is set to true.
* LIDAR plugin is now built as a shared library.
* Added CMake code for LIDAR plugin.
* Added CMake code for geotagged images plugin.
* Added remaining plugins from sitl_gazebo repo (.cpp and .h files). Have not updated CMake files yet.
* Turned the 'connect gazebo to ros' and 'connect ros to gazebo' topic names into global variables (couldn't work out how to make them global from the .world file, so this was the next best option).
* Converted gazebo_ros_interface_plugin from a model plugin to a world plugin.
* The ROS interface plugin is now attached to a static model in the world rather than being attahed to the firefly MAV.
* Added beginnings of Gazebo model for the purpose of inserting the ROS interface into the Gazebo world.
* Removed unused variables and dead comments.
* Fixed Gazebo topic name for joint_states.
* Improved the debug printing statements in .cpp files.
* Fixed topic names used in gazebo_odometry_plugin.cpp.
* Fixed typo.
* Added check to GazeboRosInterfacePlugin to make sure publisher doesn't already exist.
* Refactored arrangement of function definitions.
* Fixed include in geo_mag_declination.cpp.
* Added missing geo_mag_declination.cpp file.
* Fixed Gazebo topic name for the Gazebo plugin.
* Reverted back to commit 357ed0f254823e83e392e239a3ab7d32b595125e (Monday's commit just before .xacro files were merged).
* Added debug messages to python files, moving .xacro files from rotors_gazebo/models/rotors_description/urdf into rotors_description/models/urdf.
* Added more debug info.
* Updated debug printing.
* Updated debug printing info in GazeboMotorModel.
* Fixed up topic paths in GazeboMotorModel.
* Printing out the motor velocity topic path.
* Changed Gazebo topic name for the actual motor speed topics.
* Added print message when GazeboMavlinkInterface gets loaded.
* Specifically added the protobuf message library as a dependency to the MAVlink plugin.
* Removed SHARED qualifier for gazebo_mavlink_interface in the cmake file.
* Added geo_mag_declination from sitl_gazebo repo.
* Added mavlink header files (for mavlink plugin).
* Added Lidar, MotorSpeed and OpticalFlow messages from sitl_gazebo.
* Fixed include.
* Adjusted CMakeLists.txt for mavlink plugin.
* Changed behaviour of MAVLINK_INTERFACE variable in CMakeLists so PX4 firmware can set it to TRUE.
* Replaced gazebo mavlink interface plugin with version from sitl_gazebo.
* Changed CMakeList variable name that controls ROS dependency inclusion to 'NO_ROS'. CMakeLists.txt now checks to see if it's defined outside of it's own code (designed to be set via command-line argument).
* Modified CMakeLists whitespace formatting.
* The ROS interface plugin is no longer built if ROS_DEPENDENCY = FALSE. cmake/make builds o.k. with no ROS dependencies!
* Removed unused ROS dependency includes from odometry plugin.
* Removed un-used headers from gazebo_motor_model.h.
* Added commented code from sitl_gazebo r.e. modelling the change in propulsion on rotor due to relative air velocity. Added comments about why this code is not active.
* Finished removing ROS dependencies from GazeboMotorModel.
* Added new protobuf messages for GazeboMotorModel plugin. Half-way through removing ROS depdencies in GazeboMotorModel.
* Removing dependencies on mav_msgs package (for ease of testing purposes).
* Removed unused header include (was a ROS dependency).
* Pubs and subs are now created on first call to OnUpdate() so to be sure the ROS interface plugin has been loaded.
* Hovering example now working with new way of setting up the ROS interface plguin.
* Basic functionality of ROS->Gazebo message converter working.
* Adding ROS to Gazebo message conversion functionality.
* Refactoring in preparation for adding from ROS to Gazebo message conversion support.
* Converting GazeboControllerInterface so to have no ROS dependencies.
* Reverted GazeboControllerInterface just to test new non-singleton way of connecting messages using the ROS interface. Hovering example works.
* Removed references to singleton pattern for ROS interface plugin.
* Removed static .getInstance method to register gazebo to ROS connections, now using another message type instead.
* Merge branch 'master' into feature/px4_merge
  Pulling in changes from master.
* GazeboBagPlugin is only built if ROS is present.
* Fixed COPY function call.
* Add cmake module for finding Eigen package. Adjusting CMakeLists to build without ROS.
* Fixed paste error in .cpp file.
* Added Transform related messages and transformation publishing from the Odometry plugin.
* Fixed bug when building a new msg by pointing to parts of existing msgs, by using CopyFrom() instead.
* Added comments to ConnectToRos() helper classes.
* All converted plugins now use ConnectToRos().
* ConnectToRos() now working for multiple message types.
* Refactored method names associated with connecting Gazebo topics to ROS topics.
* Odometry messages are now being correctly published to ROS framework via AttachTo() function.
* Gazebo subscriber callback is now being called via AttachTo() function. AttachHelper() is now a member funciton.
* Commit before making AttachHelper a member function.
* Started adding a generic AttachTo() method for the ROS interface plugin. Compiling but not yet linking basic odometry message yet.
* Odometry message now being captured by ROS interface plugin and published to ROS framework. Hovering example now works again.
* Started modifying GazeboOdometryPlugin to publish Gazebo messages and removing the ROS dependencies.
* ROS interface plugin publishing actuators and link state messages to ROS platform
* Working on the conversion of Gazebo Actuators and JointState messages into ROS messages.
* GazeboMultirotorBase is now publishing Gazebo messages. Repeated Header type in protobuf messages has been extracted and is now shared between other message types, closes `#326 <https://github.com/ethz-asl/rotors_simulator/issues/326>`_. Added debug print to plugin Load() methods to see what plugins are been run by hovering example.
* Use gzlog and ROS_ERROR instead of std::cout in service callback of the gazebo octomap plugin.
* Renamed gazebo_msg_interface_plugin to gazebo_ros_interface_plugin, closes `#324 <https://github.com/ethz-asl/rotors_simulator/issues/324>`_.
* ROS message interface plugin now converts magnetic field messages and publishes to ROS.
* Add SDF tag for octomapPubTopic and octomapServiceName and load the strings in the gazebo octomap plugin.
* Added protobuf message type for magnetometer sensor. Magnetometer plugin now publishes Gazebo messages.
* Changed the name of the robot location GPS message to nav_sat_fix (since there is more than one GPS message type).
* Fixed issue with topic name differences between IMU/GPS plugins and the interface plugin.
* Converted ROS asserts to Gazebo asserts, removed ROS header file inclusions from IMU files.
* Adjusting the topic names and removing duplicate model names from namespace.
* Added TwistedStamp protobuf message type for sending ground position messages within Gazebo.
* Renamed GPS message. Gazebo GPS plugin should now emit NavSatFix messages on the Gazebo framework.
* Added GPS protobuf message type.
* Add option to publish octomap in the ServiceCallback of the gazebo_octomap_plugin.
* Modifying Gazebo GPS plugin to publish Gazebo messages instead of ROS msgs. Commit before adding GPS protobuf message.
* All fields from Gazebo IMU msg copied into ROS IMU msg.
* Working on gazebo to ROS interface plugin. Fixed bug with IMU message header types.
* New Gazebo message interface plugin is loading correctly when hover sim is launched.
* Gazebo is now outputting debug messages to the console (verbose mode is turned on through launch file).
* Fixed error where Google protobuf message indexes where outside limits.
* Added template class for new Gazebo plugin to act as message interface to both Mavlink and ROS. Code is just a template, no functionality yet implemented.
* Fixed bug with un-resolved symbol. Hovering sim now works fine, although it shouldn't be getting any IMU data anymore.
* IMU plugin is now compiling. Crashing on hover sim start due to undefined symbol.
* Removed un-used extra config variable from cmake file. IMU message type is now a custom type.
* Compiled protobuf files are now being copied into devel space, and can be included from other C++ files.
* Begun reworking IMU plugin to publish Gazebo messages. Protobuf files being built/included using CMakeLists.txt.
* Added method/class comments.
* Added namespace comment as per Google style guide.
* Fixing the order of operations in stopping the recording of a rosbag
* Comment clean-up
* Replacing Vector3Stamped with TwistStamped for ground speed publishing
* Adding precompiler checks in gps plugin to fix Gazebo API compatibility
* Possible fix for compilation error with Gazebo API version 5
* Adding the plugin to publish data from a GPS sensor on a ROS topic
* Adding a wrapper for some deprecated Gazebo API calls in sensors::GPSSensor
* Magnetometer refactoring to make use of constexpr and proper transform convention
* Creating a magnetometer plugin that is independent of Gazebo API
* Adding the magnetometer gazebo plugin
* Adding a const for initial default value for is_recording\_ in rosbag plugin
* Moving a wrapper for deprecated sdf API moved to a separate class
* Adding ability to start and stop rosbag recording on command
* Adding ability to start and stop rosbag recording on command
* Adding a wrapper class for sdf::Vector3 accessors
* Fixing warnings for deprecated SDF usage
* Fixing a couple of small bugs in mavlink interface plugin
* Refactoring mavlink interface plugin more
* Removing unfinished parts of px4 dummy controller and gazebo mavlink interface. Refactoring gazebo mavlink interface.
* Removing the unfinished wing plugin and the VTOL model
* added comments explaining octomap limitations
* corrected formatting
* improved counter and fixed typo
* cleaned up code a little, added progeress counter
* mark unseen cubes as solid
* switching to edge detection + floodfill method for making octomaps
* switching to edge detection + floodfill method for making octomaps
* Fix octomap plugin hang.
* Merge pull request `#269 <https://github.com/ethz-asl/rotors_simulator/issues/269>`_ from ethz-asl/feature/odometry_plugin_cleanup
  general cleanup of plugins and fixes for TFs
* updated comment
* removed spam
* improved IMU plugin in gazebo5
* Updated to use mavros_msgs. Optionally add mavlink_interface to gazebo models
* style fixes
* set all queue lengths to 1
* Modified CMakelist to optionally build the mavlink_interface_plugin
* removed mavros from build_depend run_depend
* Silly formating commit 2
* Silly formating commit
* Added the mavlink interface plugin to the iris model via sdf file
* Added mavlink interface plugin
* Moved mavros dependent stuff out of other plugins into mavlink interface plugin
* Removed message runtime from cmakelist
* Changes required to get posix_sitl with mavros bridge running
* Removed message runtime from cmakelist
* Merge remote-tracking branch 'origin/feature/tfdependency'
* Merge pull request `#16 <https://github.com/ethz-asl/rotors_simulator/issues/16>`_ from PX4/feature/tfdependency
  gazebo plugins: depend on tf
* gazebo plugins: depend on tf
  contributed by @devbharat
* manually apply 4f1cf03aafca38590fec45d0695ef52383e48645
* Merge remote-tracking branch 'upstream/master' into px4_nodes_upstreammerge3
* Revert "remove usage of deprecated function"
  This reverts commit 2663d9d664f0a6cb759be2f18152bdc1c47db3f9.
* remove usage of deprecated function
* update launch and xacro files to new names, update use of odometry plugin
* remove whitespace difference to upstream
* move px4 files to new directories
* Contributors: Fadri Furrer, Geoffrey Hunter, Helen Oleynikova, Jon Binney, Julius Bullinger, Michael Burri, Pavel, Thomas Gubler, Zachary Taylor, acfloria, devbharat, pvechersky, z

2.0.1 (2015-08-10)
------------------
* fixed the bag plugin and the evaluation
* Contributors: Fadri Furrer

2.0.0 (2015-08-09)
------------------
* Changed to new mav_comm messages.
* Changed default topics to be those from mav_msgs/default.h.
* Contributors: Haoyao Chen, Helen Oleynikova, Michael Burri

1.1.6 (2015-06-11)
------------------

1.1.5 (2015-06-09)
------------------
* added install targets

1.1.4 (2015-05-28)
------------------
* added std_srvs dependency

1.1.3 (2015-05-28)
------------------
* added installation of controller libraries

1.1.2 (2015-05-27)
------------------

1.1.1 (2015-04-24)
------------------
* switched from opencv to cv_bridge

1.1.0 (2015-04-24)
------------------
* initial Ubuntu package release
