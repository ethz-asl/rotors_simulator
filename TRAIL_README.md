# TRAIL RotorS

## How to use for drone landing project
1. Clone this repository and follow the instructions for compilation in `README.md`
2. Run the launch file. It will default to the Pelican model, PID gains are tuned for this.
`roslaunch rotors_gazebo mav_landing_sim.launch`
3. The simulation will automatically run. Check out the published topics with `rostopic echo`. We care about the following topics:
- `/landing_target/set_velocity`: set the target velocity
- `/landing_target/pose` and `/landing_target/twist`: get the target state
- `/pelican/command/roll_pitch_yaw_height`: send a RPY + height command to the UAV (I recommend that you set the UAV in a hover before applying RPY)
- `/pelican/odometry_sensor1/pose_with_covariance`: UAV pose from simulated odometry
4. Note that the PID gains are quite lazily tuned for yaw and height control, so feel free to tune them further. Also note that I did some bad things and reused this [mav_msgs topic](http://docs.ros.org/api/mav_msgs/html/msg/RollPitchYawrateThrust.html) for absolute yaw and height control. The `yaw` angle is mapped to `yaw_rate` and `height` is mapped to `thrust.z`.
