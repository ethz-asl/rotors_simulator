#!/usr/bin/env python
# This is needed such that int / int gives a float instead of an int
from __future__ import division

import roslib
roslib.load_manifest('rotors_evaluation')

from rosbag_tools import analyze_bag, helpers


__author__ = "Fadri Furrer, Michael Burri, Markus Achtelik"
__copyright__ = ("Copyright 2015, Fadri Furrer & Michael Burri & "
                 "Markus Achtelik, ASL, ETH Zurich, Switzerland")
__credits__ = ["Fadri Furrer", "Michael Burri", "Markus Achtelik"]
__license__ = "ASL 2.0"
__version__ = "0.1"
__maintainer__ = "Fadri Furrer"
__email__ = "fadri.furrer@mavt.ethz.ch"
__status__ = "Development"


def main():
    [ab, plot, begin_time, total_end_time, rms_calc_time, settling_radius,
     min_settled_time, first_waypoint_evaluation_delay] = helpers.initialize()

    position_error_max = 0.2  # [m]
    angular_velocity_error_max = 0.2  # [rad/s]

    # Get the begin and end times, and the set point from the waypoints.
    waypoints = ab.waypoint[0]
    bag_time_start = ab.bag_time_start.to_sec()
    bag_time_end = ab.bag_time_end.to_sec()

    settling_time = None
    print("\n")
    [begin_time, end_time] = helpers.get_evaluation_period(
        waypoints, 0, bag_time_start, bag_time_end, total_end_time)
    print("[Waypoint %d]: [%.3f, %.3f, %.3f, %.3f] at time %.3f s" % (
        0, waypoints.x[0], waypoints.y[0], waypoints.z[0],
        waypoints.yaw[0], begin_time))

    set_point_pos = analyze_bag.create_set_point(
        waypoints.x[0], waypoints.y[0], waypoints.z[0])
    set_point_pqr = analyze_bag.create_set_point(0, 0, 0)

    begin_time += first_waypoint_evaluation_delay
    print("Setting the evaluation start time %f s after the "
          "waypoint was published."
          % first_waypoint_evaluation_delay)

    rms_evaluation_start_time = begin_time
    rms_evaluation_end_time = min(begin_time + rms_calc_time, end_time)
    # Get the position RMS errors.
    rms_positions = ab.pos[0].slice(rms_evaluation_start_time,
                                    rms_evaluation_end_time)
    pos_rms_error = helpers.get_rms_position_error(
        rms_positions, set_point_pos, 0, print_output=False)

    # Get the angular velocity RMS errors.
    angular_velocities = ab.pqr[0].slice(rms_evaluation_start_time,
                                         rms_evaluation_end_time)
    pqr_rms_error = helpers.get_rms_angular_velocity_error(
        angular_velocities, set_point_pqr, 0, print_output=False)

    # Plot pose msg content if there are any pose topics.
    if plot and len(ab.pose_topics) > 0:
        x_range = [begin_time - 2, rms_evaluation_end_time + 2]
        # ab.plot_3d_trajectories()
        helpers.plot_positions(
            ab, begin_time, rms_evaluation_end_time, settling_time,
            settling_radius, set_point_pos, x_range, str(0))
        helpers.plot_angular_velocities(
            ab, begin_time, rms_evaluation_end_time, settling_time,
            x_range, str(0))

    start_collision_time = (waypoints.bag_time[0].to_sec() +
                            first_waypoint_evaluation_delay)
    if (helpers.no_collisions_occured(ab, start_collision_time,
                                      rms_evaluation_end_time)):
        print("\n")
        helpers.print_scoring(pos_rms_error, position_error_max,
                              "position RMS error", "m", [0.0, 1.5, 3.5, 5.0])
        helpers.print_scoring(pqr_rms_error, angular_velocity_error_max,
                              "angular velocity RMS error", "rad/s",
                              [0.0, 1.0, 2.0, 3.0])


if __name__ == "__main__":
    main()
