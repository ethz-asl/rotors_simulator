# Rotors 2

RotorS 2 is the ROS 2 version of the Rotor S MAV gazebo simulator by Autonomous Systems Lab @ ETH Zürich . It provides multirotor model such as Swift Pico (developed by eYantra, IIT Bombay). There is a simulated generic odometry sensor. This package also contains a basic warehouse world and launch files for the simulation.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Fortress

<h2>Installation Instructions</h2>

- Create a ROS 2 workspace `rotors_ws` and clone into the `src` folder inside it.

```
mkdir ~/rotors_ws
cd rotors_ws
git clone -b rotors2 https://github.com/arunser/rotors_simulator.git src
cd src
git clone -b ros2 https://github.com/arunser/mav_comm.git
```

- Build the packages using colcon.

```
cd rotors_ws
colcon build
source install/setup.bash
```

- To launch the Swift Pico drone in a basic world in Gazebo Fortress, use the following commands:

    - Set the `GZ_SIM_RESOURCE_PATH` environment variable to include the path to the `rotors_swift_description/models` directory.
    ```
    export GZ_SIM_RESOURCE_PATH="path_to/rotors_swift_description/models
    ```
    - To launch the Gazebo simulation for Swift Pico.

    ```
    ros2 launch rotors_swift_gazebo swift_pico_simulation.launch.py
    ```
