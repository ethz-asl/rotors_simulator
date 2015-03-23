#!/usr/bin/env python

from subprocess import call
import os


executable = os.path.abspath(__file__ + "/../") + "/eval.py"
example_bag = (os.path.abspath(__file__ + "/../../") +
               "/test_bags/waypoint_example.bag")
call([
    executable,
    "-b", example_bag,
    "--settling_radius", "0.1",
    "--min_settled_time", "3",
    "--delay_first_evaluation", "3",
    "--save_plot", "True",
    "-w", "/asymmetric_quadrotor/command/trajectory_position_yaw",
    "-W", "/asymmetric_quadrotor/collisions"])
