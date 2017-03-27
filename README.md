RotorS
===============

RotorS is a MAV gazebo simulator.
It provides some multirotor models such as the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/), the [AscTec Pelican](http://www.asctec.de/en/uav-uas-drone-products/asctec-pelican/), or the [AscTec Firefly](http://www.asctec.de/en/uav-uas-drone-products/asctec-firefly/), but the simulator is not limited for the use with these multicopters.

There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the [VI-Sensor](http://wiki.ros.org/vi_sensor), which can be mounted on the multirotor.

This package also contains some example controllers, basic worlds, a joystick interface, and example launch files.

Below we provide the instructions necessary for getting started. See RotorS' wiki for more instructions and examples (https://github.com/ethz-asl/rotors_simulator/wiki).

If you are using this simulator within the research for your publication, please cite:
```bibtex
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```
Installation Instructions
-------------------------

 1. Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-octomap-ros python-wstool python-catkin-tools
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/indigo/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 ```
 > **Note** for setups with multiple workspaces please refer to the official documentation at http://docs.ros.org/independent/api/rosinstall/html/ by replacing `rosws` by `wstool`.
 3. Get the simulator and additional dependencies

 ```
 $ cd ~/catkin_ws/src
 $ git clone git@github.com:ethz-asl/rotors_simulator.git
 $ git clone git@github.com:catkin/catkin_simple.git
 $ git clone git@github.com:ethz-asl/mav_comm.git
 $ git clone git@github.com:ethz-asl/yaml_cpp_catkin.git
 ```
  > **Note** if you want to use `wstool` you can replace the above commands with
    ```
    wstool set --git local_repo_name git@github.com:organization/repo_name.git
    ```
  > **Note** if you want to build and use the `gazebo_mavlink_interface` plugin you have to get MAVROS as an additional dependency from link below. Follow the installation instructions provided there and build all of its packages prior to building the rest of your workspace. 
    ```
    https://github.com/mavlink/mavros
    ```
 4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin init  # If you haven't done this before.
   $ catkin build
   ```

 5. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

Basic Usage
-----------

Launch the simulator with a hex-rotor helicopter model, in our case, the AscTec Firefly in a basic world.

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database.

The simulator starts by default in paused mode. To start it you can either
 - use the Gazebo GUI and press the play button
 - or you can send the following service call.

   ```
   $ rosservice call gazebo/unpause_physics
   ```

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/launch`.

The `world_name` argument looks for a .world file with a corresponding name in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds`. By default, all launch files, with the exception of those that have the world name explicitly included in the file name, use the empty world described in `basic.world`.

### Getting the multicopter to fly

To let the multicopter fly you need to generate thrust with the rotors, this is achieved by sending commands to the multicopter, which make the rotors spin.
There are currently a few ways to send commands to the multicopter, we will show one of them here.
The rest is documented [here](../../wiki) in our Wiki.
We will here also show how to write a stabilizing controller and how you can control the multicopter with a joystick.

#### Send direct motor commands

We will for now just send some constant motor velocities to the multicopter.

```
$ rostopic pub /firefly/command/motor_speed mav_msgs/Actuators '{angular_velocities: [100, 100, 100, 100, 100, 100]}'
```

> **Note** The size of the `motor_speed` array should be equal to the number of motors you have in your model of choice (e.g. 6 in the Firefly model).

You should see (if you unpaused the simulator and you have a multicopter in it), that the rotors start spinning. The thrust generated by these motor velocities is not enough though to let the multicopter take off.
> You can play with the numbers and will realize that the Firefly will take off with motor speeds of about 545 on each rotor. The multicopter is unstable though, since there is no controller running, if you just set the motor speeds.


#### Let the helicopter hover with ground truth odometry

You can let the helicopter hover with ground truth odometry (perfect state estimation), by launching:

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

#### Create an attitude controller

**TODO(ff):** `Write something here.`

#### Usage with a joystick

Connect a USB joystick to your computer and launch the simulation alongside ROS joystick driver and the RotorS joystick node:
```
$ roslaunch rotors_gazebo mav_with_joy.launch mav_name:=firefly world_name:=basic
```

Depending on the type of joystick and the personal preference for operation, you can assign the axis number using the `axis_<roll/pitch/thrust>_` parameter and the axis direction using the `axis_direction_<roll/pitch/thrust>` parameter.

#### Usage with a keyboard

First, perform a one-time setup of virtual keyboard joystick as described here: https://github.com/ethz-asl/rotors_simulator/wiki/Setup-virtual-keyboard-joystick.

Launch the simulation with the keyboard interface using the following launch file:
```
$ roslaunch rotors_gazebo mav_with_keyboard.launch mav_name:=firefly world_name:=basic
```

If everything was setup correctly, an additional GUI should appear with bars indicating the current throttle, roll, pitch, and yaw inputs. While this window is active, the Arrows and W, A, S, D keys will generate virtual joystick inputs, which can then be processed by the RotorS joystick node in the same way as real joystick commands.

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionlity, as there are the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.

Extended Wind Plugin
-------------------------

The existing wind plugin in RotorS has been extended to allow the use of a custom, static wind field for a given world. This is done by enabling the wind plugin macro in the aircraft base file (in our case `techpod_base.xacro`, which is found in `rotors_simulator/rotors_description/urdf`), and feeding it the correct parameters and files. The wind grid and values are generated using a suitable model, and saved in a text file read by the plugin.

#### Grid specifications

The grid used to define the wind field must be equidistant in x, respectively y-direction. The points in z-direction are distributed with upwards linearly increasing distance (spacing factors can be tuned as desired). The grid is terrain-following, meaning that all the points in the lower z-layer have a constant altitude offset from the terrain.

#### Wind field text file format

The text file contains information about the grid geometry as well as the wind values at each grid point. The data needed in the text file consists of:

  1. Smallest x-coordinate of the grid `min_x`, of type float, in [m].

  2. Smallest y-coordinate of the grid `min_y`, of type float, in [m].

  3. Number of grid points in x-direction `n_x`, of type integer.

  4. Number of grid points in y-direction `n_y`, of type integer.

  5. Resolution in x-direction `res_x`, of type float, in [m].

  6. Resolution in y-direction `res_y`, of type float, in [m].

  7. Spacing factors in z-direction stored in `vertical_spacing_factors`, an (n_z)-dimensional array of float values between 0 for the lowest and 1 for the highest point.
  8. The altitude of each grid point contained in the lower x-y plane, stored in `bottom_z`, an (n_x*n_y)-dimensional array of float values, in [m].
  > **Note**: Element [0] is the grid corner point with the lowest x and y-coordinates, and the array is filled counting up in x-direction first, then in y-direction (such that a point with indices i,j corresponds to the (i+j*n_x)th element of the array).

  9. Similarly, the altitude of each grid point contained in the upper x-y plane, stored in `top_z`, an (n_x*n_y)-dimensional array of float values, in [m].

  10. The x-component of the wind speed for each grid point, stored in `u`, an (n_x*n_y*n_z)-dimensional array of float values, in [m/s].
  > **Note**: Element [0] is the grid corner point with the lowest x, y and z-coordinates, and the array is filled counting up in x-direction first, then in y-direction and finally in z-direction (such that a point with indices i,j,k corresponds to the (i + j*n_x + k*n_x*n_y)th element of the array).

  11. Similarly, the y-component of the wind speed stored in the array `v`, and its z-component stored in `w`.

The order in which the data is saved in the text file is not relevant, but the format must comply with the following requirements:

  1. In the first line of the text file, the name of one of the 12 needed data followed directly by a colon (e.g., `vertical_spacing_factors:`)

  2. In the following line, the corresponding data. If multiple values are needed, they must be separated by a space (e.g., `0.0 0.025641 0.051282 0.076923 ...`)

  3. The rest of the data follows the same format.

An example wind field text file can be seen for the hemicylindrical world at `$(find rotors_gazebo)/models/hemicyl` (placed in the same folder as the world model for clarity and convenience).

#### Wind Plugin Macro

Here is an example of the plugin macro to be added in the base file, containing numerous necessary user-defined variables:

```
<xacro:wind_plugin_macro
  namespace="${namespace}"
  xyz_offset="0 0 0"
  wind_direction="1 0 0"
  wind_force_mean="0.0"
  wind_gust_direction="0 1 0"
  wind_gust_duration="0.0"
  wind_gust_start="0.0"
  wind_gust_force_mean="0.0"
  wind_speed_mean="5.0"
  custom_static_wind_field="true"
  custom_wind_field_path="$(find rotors_gazebo)/models/hemicyl/custom_wind_field.txt">
</xacro:wind_plugin_macro>
```
All parameters needed in the macro as well as their units are specified in the `component_snippets.xacro` file.
The four relevant values for the use of the extended plugin are `wind_direction` and `wind_speed_mean`, which specify the default constant wind field when the aircraft is flying outside of the custom wind field region, the boolean `custom_static_wind_field` which, when set to `true`, enables the extended functionality, as well as the string `custom_wind_field_path` which describes the path (from `~/.ros`) to the text file specifying the grid and wind field.

#### Functioning
In brief, the plugin works in distinct steps:

  1. Load the plugin and read the text file once, saving the data.

  2. During update event:

   2.1. Locate the aircraft and see whether it is flying within the specified wind field bounds.

    2.2. If so, identify the grid points forming the vertices of the enclosing cell and extract their wind values. If not, set the wind velocity to the user-defined default value.

    2.3. Interpolate linearly in z, x and y-directions to find the wind velocity at the aircraft position.

    2.4. Publish the wind velocity in a wind speed message.

