Installation Instructions - Ubuntu 18.04 with ROS Melodic
---------------------------------------------------------

1. Install and initialize ROS kinetic desktop full, dependencies for building packages, catkin-tools, and wstool. For further information check out http://wiki.ros.org/melodic/Installation/Ubuntu, also check ROS page for potential changes of the key used below:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
$ sudo rosdep init
$ rosdep update
$ sudo apt install python-rosinstall python-rosinstall-generator python-catkin-tools python-wstool build-essential
```

2. Create, initialize and configure a ROS Workspace (if you don't have one already that you want to use with rotors)

 ```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin init
$ catkin config --merge-devel
$ catkin config --extend /opt/ros/melodic
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
 ```
3. Get RotorS and additional dependencies

```
$ cd ~/catkin_ws/src
$ git clone -b feature/fw_hil_rotors https://github.com/ethz-asl/rotors_simulator.git
$ git clone https://github.com/ethz-asl/mav_comm.git
$ git clone https://github.com/catkin/catkin_simple.git
```
a) If you want upstream mavros, do:

```
$ cd ~/catkin_ws
$ wstool init src
$ rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall
$ rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
$ wstool merge -t src /tmp/mavros.rosinstall
$ wstool update -t src -j4
$ rosdep install --from-paths src --ignore-src -y
$ ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```
b) To use asl-ethz/fw_mavros version (not tested yet, adapted from https://github.com/ethz-asl/fw_mavros), do:

```
$ pip install future
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-geographic-msgs libgeographic-dev ros-kinetic-diagnostic-updater ros-kinetic-tf2-ros ros-kinetic-tf2-eigen ros-kinetic-angles ros-kinetic-eigen-conversions
$ sudo geographiclib-get-geoids egm96-5
$ cd ~/catkin_ws/src
$ git clone git@github.com:mavlink/mavlink-gbp-release
$ cd ~/catkin_ws/src/mavlink-gbp-release
$ git checkout debian/melodic/mavlink
$ cd ~/catkin_ws/src
$ git clone git@github.com:ethz-asl/fw_mavros
$ catkin build mavros
```
4. Build

```
$ cd ~/catkin_ws
$ catkin build
```

