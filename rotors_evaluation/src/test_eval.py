#!/usr/bin/env python

from subprocess import call
import os


__author__ = "Fadri Furrer, Michael Burri, Markus Achtelik"
__copyright__ = ("Copyright 2015, Fadri Furrer & Michael Burri & "
                 "Markus Achtelik, ASL, ETH Zurich, Switzerland")
__credits__ = ["Fadri Furrer", "Michael Burri", "Markus Achtelik"]
__license__ = "ASL 2.0"
__version__ = "0.1"
__maintainer__ = "Fadri Furrer"
__email__ = "fadri.furrer@mavt.ethz.ch"
__status__ = "Development"


executable = os.path.abspath(__file__ + "/../") + "/waypoints_eval.py"
example_bag = (os.path.abspath(__file__ + "/../../") +
               "/test_bags/waypoint_example.bag")
call([
    executable,
    "-b", example_bag,
    "--settling_radius", "0.1",
    "--min_settled_time", "3",
    "--delay_first_evaluation", "5",
    "--save_plot", "True",
    "--mav_name", "asymmetric_quadrotor"])
