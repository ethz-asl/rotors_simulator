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

    settling_time_max = 10.0  # [s]
    position_error_max = 0.2  # [m]
    angular_velocity_error_max = 0.2  # [rad/s]

    # Get the begin and end times, and the set point from the waypoints.
    waypoints = ab.waypoint[0]
    bag_time_start = ab.bag_time_start.to_sec()
    bag_time_end = ab.bag_time_end.to_sec()
    list_settling_time = []
    list_pos_rms = []
    list_pqr_rms = []

    # Iterate over waypoints, and create evaluations for each waypoint.
    for index in range(len(waypoints.x)):
        settling_time = None
        print("\n")
        [begin_time, end_time] = helpers.get_evaluation_period(
            waypoints, index, bag_time_start, bag_time_end, total_end_time)
        print("[Waypoint %d]: [%.3f, %.3f, %.3f, %.3f] at time %.3f s" % (
            index, waypoints.x[index], waypoints.y[index], waypoints.z[index],
            waypoints.yaw[index], begin_time))

        # Get all positions for the evaluation period.
        positions = ab.pos[0].slice(begin_time, end_time)

        set_point_pos = analyze_bag.create_set_point(
            waypoints.x[index], waypoints.y[index], waypoints.z[index])
        set_point_pqr = analyze_bag.create_set_point(0, 0, 0)

        # Treat the first waypoint specially, as the MAV will most likely still
        # be in collision (with the ground), as the first waypoint gets
        # published. You can specify the delay of the evaluation with the
        # first_waypoint_evaluation_delay option.
        if index == 0:
            begin_time += first_waypoint_evaluation_delay
            print("Setting the first evaluation start time %f s after the "
                  "first waypoint was published.\n"
                  % first_waypoint_evaluation_delay)
            print("[Waypoint %d]: Settling time for this waypoint not\n"
                  "              considered." % index)

            rms_evaluation_start_time = begin_time
            rms_evaluation_end_time = min(begin_time + rms_calc_time, end_time)
            # Get the position RMS errors.
            rms_positions = ab.pos[0].slice(rms_evaluation_start_time,
                                            rms_evaluation_end_time)
            pos_rms_error = helpers.get_rms_position_error(
                rms_positions, set_point_pos, index)

            # Get the angular velocity RMS errors.
            angular_velocities = ab.pqr[0].slice(rms_evaluation_start_time,
                                                 rms_evaluation_end_time)
            pqr_rms_error = helpers.get_rms_angular_velocity_error(
                angular_velocities, set_point_pqr, index)
        else:
            # Get the time at which the MAV stayed for at least
            # min_settled_time seconds within a ball of settling_radius meters
            # around set_point_pos.
            settling_time = helpers.get_settling_time(
                positions, set_point_pos, settling_radius, min_settled_time,
                index)
            if settling_time is not None and settling_time < settling_time_max:
                rms_evaluation_start_time = begin_time + settling_time
                rms_evaluation_end_time = min(begin_time + settling_time +
                                              rms_calc_time, end_time)
                # Get the position RMS errors
                rms_positions = ab.pos[0].slice(
                    rms_evaluation_start_time, rms_evaluation_end_time)
                pos_rms_error = helpers.get_rms_position_error(
                    rms_positions, set_point_pos, index)

                # Get the angular velocity RMS errors
                angular_velocities = ab.pqr[0].slice(
                    rms_evaluation_start_time, rms_evaluation_end_time)
                pqr_rms_error = helpers.get_rms_angular_velocity_error(
                    angular_velocities, set_point_pqr, index)

                list_settling_time.append(settling_time)
                list_pos_rms.append(pos_rms_error)
                list_pqr_rms.append(pqr_rms_error)
            else:
                print("[Waypoint %d]: System didn't settle  in %f seconds -- "
                      "inserting 101 % of defined maximum values."
                      % (index, settling_time_max))
                list_settling_time.append(settling_time_max * 1.01)
                list_pos_rms.append(position_error_max * 1.01)
                list_pqr_rms.append(angular_velocity_error_max * 1.01)

        # Plot pose msg content if there are any pose topics.
        if plot and len(ab.pose_topics) > 0:
            x_range = [begin_time - 2, rms_evaluation_end_time + 2]
            # ab.plot_3d_trajectories()
            helpers.plot_positions(
                ab, begin_time, rms_evaluation_end_time, settling_time,
                settling_radius, set_point_pos, x_range, str(index))
            helpers.plot_angular_velocities(
                ab, begin_time, rms_evaluation_end_time, settling_time,
                x_range, str(index))

    average_settling_time = helpers.calculate_average(list_settling_time)
    average_pos_rms = helpers.calculate_average(list_pos_rms)
    average_pqr_rms = helpers.calculate_average(list_pqr_rms)

    start_collision_time = (waypoints.bag_time[0].to_sec() +
                            first_waypoint_evaluation_delay)
    if (helpers.no_collisions_occured(ab, start_collision_time,
                                      rms_evaluation_end_time)):
        print("\n")
        helpers.print_scoring(average_settling_time, settling_time_max,
                              "settling time", "s", [0.0, 1.5, 3.5, 5.0])
        helpers.print_scoring(average_pos_rms, position_error_max,
                              "position RMS error", "m", [0.0, 1.5, 3.5, 5.0])
        helpers.print_scoring(average_pqr_rms, angular_velocity_error_max,
                              "angular velocity RMS error", "rad/s",
                              [0.0, 1.0, 2.0, 3.0])


if __name__ == "__main__":
    main()
