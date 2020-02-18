# Installation Instructions - Ubuntu 18.04 with ROS Melodic

### 1. ROS 
Install and initialize ROS melodic desktop full, dependencies for building packages, catkin-tools, and wstool. For further information check out http://wiki.ros.org/melodic/Installation/Ubuntu, also check ROS page for potential changes of the key used below:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full
$ sudo rosdep init
$ rosdep update
$ sudo apt install python-rosinstall python-rosinstall-generator python-catkin-tools python-wstool build-essential
```

### 2. Create Workspace
Create, initialize and configure a ROS Workspace (if you don't have one already that you want to use with rotors)

 ```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin init
$ catkin config --merge-devel
$ catkin config --extend /opt/ros/melodic
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
 ```
### 3. RotorS 
Get RotorS and additional dependencies

```
$ cd ~/catkin_ws/src
$ git clone -b feature/fw_hil_rotors https://github.com/ethz-asl/rotors_simulator.git
$ git clone https://github.com/ethz-asl/mav_comm.git
$ git clone https://github.com/catkin/catkin_simple.git
```
If you want upstream mavros, do:

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
To use asl-ethz/fw_mavros version (not tested yet, adapted from https://github.com/ethz-asl/fw_mavros), do:

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
Note: HIL simulation currently requires the extended ASLUAV mavlink dialect defined here: https://github.com/ethz-asl/fw_mavlink/blob/55397e58ebba631165b0528eb75d2ffec0ccf919/message_definitions/v1.0/ASLUAV.xml. After the above installation steps, replace the ASLUAV.xml in /catkin_ws/src/mavlink-gbp-release/message_definitions/v1.0/ with the ASLUAV.xml found under the link given above... A bit hacky at the moment, tb improved in the future. You can then proceed to build the catkin workspace:

### 4. Build
```
$ cd ~/catkin_ws
$ catkin build
$ source ~/catkin_ws/devel/setup.bash
```

# Basic Usage

### 1. Setup
Don't forget to always overlay workspace on top of environment (active only in current shell)

```
$ cd ~/catkin_ws
$ source ~/catkin_ws/devel/setup.bash
```

Alternatively, if you don't want to do this every time, add setup directly to bash sessions

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Run Simulation
Hook up Pixhawk to your computer via USB (should appear as /dev/ttyACM0 on linux/ubuntu). It needs to run in HIL-mode and have the proper firmware and airframe installed. Make sure that the gazebo_mavlink_interface.cpp plugin of Gazebo is configured to match the control channel assignment as defined in the PX4 mixer file of the airframe set on the Pixhawk. This can be checked (or changes made) in the respective \*.xacro file of the uav/robot under the <plugin filename="librotors_gazebo_mavlink_interface.so" .../> tag (\*.xacro files are located in your workspace under /src/rotors_simulator/rotors_description/urdf/your_robot.xacro). Wait until the Pixhawk booted before you start the simulation.

To launch a fixed-wing simulation, go to your catkin workspace and invoke (choosing one of the param values at a time...):

"roslaunch rotors_gazebo fixed_wing_hil.launch verbose:={false, true} world_name:={fw_playground, yosemite, davos, hinwil} uav_name:={techpod_TJ_flex, techpod_TJ, techpod_X_flex, techpod_X, list_not_complete} spawn_tc:={false, true} enable_wind:={true, false} record_rosbag:={false, true} rosbag_path:={"/path/to/somewhere/"}"

where "verbose" can be used to enable debug output (e.g. sim-rate, message statistics etc), "world_name" sets the world in which the UAV spawns and "uav_name" is used to select the UAV. "spawn_tc" further controls if tracking cameras are placed (ground view) and "enable_wind" indicates if the wind specified in the uav's *.xacro is enabled. Defaults are the first options in each brace. In your workspace e.g.invoke:

```
$ roslaunch rotors_gazebo fixed_wing_hil.launch world_name:=fw_playground uav_name:=techpod_X spawn_tc:=true enable_wind:=true verbose:=false

```
Upon start of ROS/Gazebo with the desired world and UAV, the simulation should start to communicate with the Pixhawk. The connection is established by the gazebo_mavlink_interface plugin of the UAV which sets up a mavlink connection (its configuration can again be found/changed in the robot's \*.xacro). The UAV/simulation obtains actuator commands via the mavlink message HIL_ACTUATOR_CONTROLS. The mavlink messages HIL_SENSOR (for sensor-level hil) or HIL_STATE_QUATERNION (state-level hil) and HIL_GPS are, amongst others, sent back to the Pixhawk to provide feedback for control.

### 3. Notes
- RESTARTING the simulation: In case you face problems with the simulation (e.g. UAV crashes, Pixhawk lost connection, Gazebo crashed etc.), you need to restart it. Since the Pixhawk's estimator doesn't like timouts of sensor data, we suggest to also reboot the pixhawk when restarting the sim, e.g. disconnect it and start over from point 2 above. Make sure GQC doesn't run when you start the sim as it might block the connection to the Pixhawk (Sim acts as proxy between QGC and Pixhawk).
- As of now, there is no respawn functionality in HIL in case the UAV e.g. crashes (this would mess with the Pixhawk's state estimator). Therefore, either restart the simulation, see above, or apply forces and torques to the UAV via Gazebo GUI (right-click on robot -> Add forces/torques) to e.g. recover an UAV that flipped upside-down.
- The listed sets of parameters include tested and working UAV/world combinations. More UAVs/worlds will be tested or added in the future. 
- In the "davos" world, the UAV currently spawns in mid-air and, subsequently, crashes into the ground -> use gazebos 'follow' function to find the UAV again.
- All "techpod" versions require an AAERTFF mixer on the Pixhawk to work properly.
- Once the simulation is running, you can start QGC to introspect e.g. the telemetry from the sim. Make sure QGC connects to the Pixhawk only via UDP (and not serial as this interferes with Gazebo). To this end, only select "UDP" in QGC under General->AutoConnect. If you restart the simulation, make sure to quit QGC first because it would block the connection between the sim and Pixhawk
- Streams of simulated cameras and other signals published as ROS messages can be displayed with rqt.
- If your robot depends on *.yaml files, for some reason you need to mount a camera on the robot for the yaml-parser to work - absolutely no idea why this is, seems completely unrelated
- Don't use special characters within \*.xacro/\*.urdf files, not even within comments. (didn't launched when there was a superscript on a unit in the comments)
