[![Build Status](https://travis-ci.org/gsilano/CrazyS.svg?branch=master)](https://travis-ci.org/gsilano/CrazyS)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

CrazyS
===============

CrazyS is an extension of the ROS package [RotorS](https://github.com/ethz-asl/rotors_simulator), aimed to modeling, developing and integrating the [Crazyflie 2.0](https://www.bitcraze.io/crazyflie-2/) nano-quadcopter in the physics based simulation environment Gazebo. The contribution can be also considered as a reference guide for expanding the RotorS functionalities in the UAVs field, by facilitating the integration of new aircrafts. 

Such simulation platform allows to understand quickly the behavior of the flight control system by comparing and evaluating different indoor and outdoor scenarios, with a details level quite close to reality. The proposed extension expands RotorS capabilities by considering the Crazyflie 2.0 physical model and its flight control system, as well (the [2018.01.1](https://github.com/bitcraze/crazyflie-firmware/releases/tag/2018.01.1) firware release).

A simple case study is considered (`crazyflie2_hovering_example.launch`) in order to show how the package works and the validity of the employed dynamical model together the control architecture of the quadcopter.

The code is released under Apache license, thus making it available for scientific and educational activities.

The platform has been developed by using Ubuntu 16.04 and the Kinetic Kame version of ROS. Although the platform is fully compatible with Indigo Igloo version of ROS and Ubuntu 14.04, such configuration is not recommended since the ROS support will close in April 2019.

Below we provide the instructions necessary for getting started. See [CrazyS' wiki](https://github.com/gsilano/CrazyS/wiki) for more instructions and examples.

If you are using this simulator within the research for your publication, please take a look at the [Publications page](https://github.com/gsilano/CrazyS/wiki/Publications). The page contains core papers and all linked works (using the platform).

Installation Instructions - Ubuntu 16.04 with ROS Kinetic
---------------------------------------------------------
 1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
$ sudo sh -c ’echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list’
$ sudo apt-key adv --keyserver hkp://ha.pool.skskeyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python-rosinstall pythonrosinstall-
generator python-wstool build-essential
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ catkin init
 $ git clone https://github.com/gsilano/CrazyS.git
 $ git clone https://github.com/gsilano/mav_comm.git
 $ cd ~/catkin_ws/src/mav_comm & git checkout crazys
 $ rosdep update
 $ cd ~/catkin_ws
 $ rosdep install --from-paths src -i
 $ catkin build
 ```

  > **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin build
   ```

 4. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```
Installation Instructions - Ubuntu 14.04 with ROS Indigo
--------------------------------------------------------

 1. Install and initialize ROS indigo desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c ’echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list’
 $ sudo apt-key adv --keyserver hkp://ha.pool.skskeyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
 $ sudo apt-get update
 $ sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-octomap-ros python-wstool python-catkin-tools protobuf compiler libgoogle-glog-dev
 $ sudo rosdep init
 $ rosdep update
 $ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
 $ source ~/.bashrc
 $ sudo apt-get install python-rosinstall
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ catkin init
 ```
 > **Note** for setups with multiple workspaces please refer to the [official documentation](http://docs.ros.org/independent/api/rosinstall/html/) by replacing `rosws` by `wstool`.
 3. Get the simulator and additional dependencies

 ```
 $ cd ~/catkin_ws/src
 $ git clone https://github.com/gsilano/CrazyS.git
 $ git clone https://github.com/gsilano/mav_comm.git
 $ cd ~/catkin_ws/src/mav_comm & git checkout crazys
 $ rosdep update
 $ cd ~/catkin_ws
 $ rosdep install --from-paths src -i
 ```
  > **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

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
   > **Note** if you are getting errors related to "future" package, you may need python future:
    ```
    sudo apt-get install python-pip
    pip install --upgrade pip
    pip install future
    ```

 5. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```
Basic Usage
-----------

Launching the simulation is quite simple, so as customizing it: it is enough to run in a terminal the command

```
$ roslaunch rotors_gazebo crazyflie2_hovering_example.launch
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database. To avoid any problems when starting the simulation for the first time, you may run the `gazebo` command in the terminal line.

By default the state estimator is disabled since on-board Crazyflie's sensors are replaced by the odometry one. For running the simulation by taking into account the Crazyflie's IMU and the complementary filter, it is enough to give a command that turns on the flag `enable state estimator`:

```
$ roslaunch rotors_gazebo crazyflie2_hovering_example.launch enable_state_estimator:=true
```

The visual outcome will see the nano-quadcopter taking off after 5s (time after which the hovering example node publishes the trajectory to follow) and flying one meter above the ground, at the same time keeping near to zero the position components along x and y-axis.

The whole process is the following: the desired trajectory coordinates (x_r, y_r, z_r and \psi_r) are published by the `hovering_example` node on the topic `command/trajectory`, to whom the `position_controller` node (i.e., the Crazyflie controller) is subscribed. The drone state (`odometry_sensor1/odometry` topic) and the references are used to run the control strategy designed for the position tracking. The outputs of the control algorithm consist into the actuation commands (\omega_1, \omega_2, \omega_3 and \omega_4) sent to Gazebo (`command/motor_speed`) for the physical simulation and the corresponding graphical rendering, so to visually update the aircraft position and orientation. When the state estimator is turned off, the drone orientation (\phi_k, \theta_k and \psi_k) and angular velocities (p_k, q_k and r_k) published on the topic odometry are replaced by the ideal values coming from the odometry sensor.

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/CrazyS/rotors_gazebo/launch`. Suche scenarios are better explained in the [RotorS](https://github.com/ethz-asl/rotors_simulator) repository.

The `world_name` argument looks for a .world file with a corresponding name in `~/catkin_ws/src/CrazyS/rotors_gazebo/worlds`. By default, all launch files, with the exception of those that have the world name explicitly included in the file name, use the empty world described in `basic.world`.

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionlity, as there are the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.

Bugs & Feature Requests
--------------

Please report bugs and request features by using the [Issue Tracker](https://github.com/gsilano/CrazyS/issues). Furthermore, please see
the [Contributing.md](https://github.com/gsilano/CrazyS/blob/master/CONTRIBUTING.md) file if you plan to help us to improve 
CrazyS features.

YouTube videos
--------------

In this section a video providing the effectiveness of the platform and how it works is reported. Further videos can be found in the related YouTube channel. Have fun! :)

[![CrazyS, an exntension of the ROS pakcage RotrS aimed to modeling, developing and integrating the Crazyflie 2.0 nano-quadcopter](https://github.com/gsilano/CrazyS/wiki/img/img_YouTube_MED18.png)](https://youtu.be/qsrYCUSQ-S4 "CrazyS, an exntension of the ROS pakcage RotrS aimed to modeling, developing and integrating the Crazyflie 2.0 nano-quadcopter")

