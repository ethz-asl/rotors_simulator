"""Helper functions for the automatic evaluation of RotorS datasets."""

import optparse
from rosbag_tools import analyze_bag


__author__ = "Fadri Furrer, Michael Burri, Markus Achtelik"
__copyright__ = ("Copyright 2015, Fadri Furrer & Michael Burri & "
                 "Markus Achtelik, ASL, ETH Zurich, Switzerland")
__credits__ = ["Fadri Furrer", "Michael Burri", "Markus Achtelik"]
__license__ = "ASL 2.0"
__version__ = "0.1"
__maintainer__ = "Fadri Furrer"
__email__ = "fadri.furrer@mavt.ethz.ch"
__status__ = "Development"


def get_score(evaluated_variable, max_variable, scores):
    """
    Calculate the score from the evaluated_variable.

    Args:
        evaluated_variable: Float of the variable to be scored.
        max_varibale: Float of the maximum value that needs to be undershoot to
                      score.
        scores: List of Floats of the returned scores (length should be 4).
    Returns:
        The score, that one gets for the evaluated_variable.
    """
    if evaluated_variable > max_variable:
        return scores[0]
    elif evaluated_variable > 0.5 * max_variable:
        return scores[1]
    elif evaluated_variable > 0.1 * max_variable:
        return scores[2]
    else:
        return scores[3]


def initialize():
    """ Read the command line params, and create an analyze object. """
    default_twist_topic = "/ground_truth/twist"
    default_pose_topic = "/ground_truth/pose"
    default_save_plots = "false"
    default_show_plots = "true"
    default_first_waypoint_delay = 5  # [s]
    default_begin_time = 0.0
    default_end_time = 1000.0
    default_rms_calc_time = 10.0
    default_settling_radius = 0.1
    default_min_settled_time = 3
    default_mav_name = ""
    default_motor_velocity_topic = "/motors"
    default_waypoint_topic = "/command/trajectory"
    default_wrench_topic = "/wrench"

    parser = optparse.OptionParser(
        """usage: %prog -b bagfile -p topic1 -t topic2

        e.g.:
        %prog -b /home/username/.ros/firefly.bag
              --mav_name firefly
        or
        %prog -b /Users/username/.ros/hummingbird.bag
              --mav_name hummingbird --save_plots True
              --delay_first_evaluation 3
        """)
    parser.add_option(
        "-b", "--bagfile",
        dest="bagfile",
        type="string",
        help="Specify the bag file you want to analyze, with the path.")
    parser.add_option(
        "-p", "--pose_topic",
        dest="pose_topic",
        default=default_pose_topic,
        type="string",
        help="The pose topic that you want to extract from the bag file.")
    parser.add_option(
        "-t", "--twist_topic",
        dest="twist_topic",
        default=default_twist_topic,
        type="string",
        help="The twist topic that you want to extract from the bag file.")
    parser.add_option(
        "-s", "--save_plots",
        action="store_true",
        dest="save_plots",
        default=default_save_plots,
        help="Store all the plots in the current directory.")
    parser.add_option(
        "--prefix",
        dest="prefix",
        default=None,
        type="string",
        help="The prefix of the output png files.")
    parser.add_option(
        "-d", "--plot",
        dest="plot",
        default=default_show_plots,
        type="string",
        help="Set to True if you want the data plotted.")
    parser.add_option(
        "--begin_time",
        dest="begin",
        default=default_begin_time,
        type="float",
        help="Start time of the evaluated data.")
    parser.add_option(
        "-e", "--end_time",
        dest="end",
        default=default_end_time,
        type="float",
        help="End time of the evaluated data.")
    parser.add_option(
        "--rms_calc_time",
        dest="rms_calc_time",
        default=default_rms_calc_time,
        type="float",
        help="The length of the RMS evaluation period.")
    parser.add_option(
        "--settling_radius",
        dest="settling_radius",
        default=default_settling_radius,
        type="float",
        help="The radius of the bounding sphere where the system should "
             "settle.")
    parser.add_option(
        "--min_settled_time",
        dest="min_settled_time",
        default=default_min_settled_time,
        type="float",
        help="The minimal time for which the system should stay bounded.")
    parser.add_option(
        "-m", "--motor_velocity_topic",
        dest="motor_velocity_topic",
        default=default_motor_velocity_topic,
        type="string",
        help="The motor_velocity topic that you want to extract from the bag "
             "file.")
    parser.add_option(
        "-w", "--waypoint_topic",
        dest="waypoint_topic",
        default=default_waypoint_topic,
        type="string",
        help="The waypoint topic that you want to extract from the bag file.")
    parser.add_option(
        "-W", "--wrench_topic",
        dest="wrench_topic",
        default=default_wrench_topic,
        type="string",
        help="The wrench topic that you want to extract from the bag file.")
    parser.add_option(
        "-D", "--delay_first_evaluation",
        dest="first_waypoint_delay",
        default=default_first_waypoint_delay,
        type="float",
        help="The time when the evaluation should start after the first "
             "waypoint got published.")
    parser.add_option(
        "-n", "--mav_name",
        dest="mav_name",
        default=default_mav_name,
        type="string",
        help="The name of your MAV (should correspond to the namespace).")

    (options, args) = parser.parse_args()
    if not options.bagfile:
        parser.error('Bagfile not given.')
    mav_name = options.mav_name
    topic_prefix = mav_name
    save_plots = options.save_plots
    prefix = options.prefix
    bagfile = options.bagfile
    pose_topics = analyze_bag.create_topic_list(
        topic_prefix + options.pose_topic)
    twist_topics = analyze_bag.create_topic_list(
        topic_prefix + options.twist_topic)
    motor_velocity_topics = analyze_bag.create_topic_list(
        topic_prefix + options.motor_velocity_topic)
    waypoint_topics = analyze_bag.create_topic_list(
        topic_prefix + options.waypoint_topic)
    wrench_topics = analyze_bag.create_topic_list(
        topic_prefix + options.wrench_topic)
    plot = True if (options.plot and options.plot.lower() == 'true') else False
    if save_plots:
        plot = True
    begin_time = options.begin
    total_end_time = options.end
    rms_calc_time = options.rms_calc_time
    settling_radius = options.settling_radius
    min_settled_time = options.min_settled_time
    first_waypoint_evaluation_delay = options.first_waypoint_delay

    # Create a new Analyze bag object, to do the evaluation on.
    ab = analyze_bag.AnalyzeBag(bag_path_name=bagfile, save_plots=save_plots,
                                prefix=prefix)

    # Add all the topics of the different message types.
    for pose_topic in pose_topics:
        ab.add_pose_topic(pose_topic)
    for twist_topic in twist_topics:
        ab.add_twist_topic(twist_topic)
    for motor_velocity_topic in motor_velocity_topics:
        ab.add_motor_velocity_topic(motor_velocity_topic)
    for waypoint_topic in waypoint_topics:
        ab.add_waypoint_topic(waypoint_topic)
    for wrench_topic in wrench_topics:
        ab.add_wrench_topic(wrench_topic)
    if len(ab.topics):
        ab.extract_messages()

    return [ab, plot, begin_time, total_end_time, rms_calc_time,
            settling_radius, min_settled_time, first_waypoint_evaluation_delay]


def get_evaluation_period(waypoints, index, bag_time_start, bag_time_end,
                          total_end_time):
    begin_time = waypoints.bag_time[index].to_sec() - bag_time_start
    # Find the end time of the evaluation.
    if index + 1 < len(waypoints.x):
        end_time = waypoints.bag_time[index + 1].to_sec() - bag_time_start
    else:
        end_time = min(total_end_time, bag_time_end)
    if (bag_time_end < end_time):
        print("[Waypoint %d]: Stopping evaluation early, as the "
              "bag file ends." % index)
    return [begin_time, end_time]


def get_settling_time(positions, set_point, radius, min_time, index,
                      print_output=True):
    settling_time = analyze_bag.settling_time(
        set_point, positions, radius, min_time)
    if settling_time is not None:
        if print_output:
            print("[Waypoint %d]: Settling time: %.3f s"
                  % (index, settling_time))
        return settling_time
    else:
        if print_output:
            print("[Waypoint %d]: System didn't settle." % index)
        return None


def get_rms_position_error(positions, position_set_point, index,
                           print_output=True):
    pos_rms_error = analyze_bag.xyz_rms_error(position_set_point, positions)
    if print_output:
        print("[Waypoint %d]: Position RMS error: %.3f m"
              % (index, pos_rms_error))
    return pos_rms_error


def get_rms_angular_velocity_error(angular_velocities,
                                   angular_velocity_set_point, index,
                                   print_output=True):
    pqr_rms_error = analyze_bag.xyz_rms_error(angular_velocity_set_point,
                                              angular_velocities)
    if print_output:
        print("[Waypoint %d]: Angular velocity RMS error: %.3f rad/s"
              % (index, pqr_rms_error))
    return pqr_rms_error


def plot_positions(analyze_bag, start_time, end_time, settling_time,
                   settling_radius, set_point_position, x_range, plot_suffix):
    try:
        absolute_settling_time = settling_time + start_time
    except:
        absolute_settling_time = None
    analyze_bag.plot_positions(
        start_time=start_time,
        end_time=end_time,
        settling_time=absolute_settling_time,
        x_range=x_range,
        plot_suffix=plot_suffix)
    analyze_bag.plot_position_error(
        set_point=set_point_position,
        settling_radius=settling_radius,
        start_time=start_time,
        end_time=end_time,
        settling_time=absolute_settling_time,
        x_range=x_range,
        y_range=[0, 1],
        plot_suffix=plot_suffix)


def plot_angular_velocities(analyze_bag, start_time, end_time, settling_time,
                            x_range, plot_suffix):
    try:
        absolute_settling_time = settling_time + start_time
    except:
        absolute_settling_time = None
    analyze_bag.plot_angular_velocities(
        start_time=start_time,
        end_time=end_time,
        settling_time=absolute_settling_time,
        x_range=x_range,
        plot_suffix=plot_suffix,
        y_range=[-1.5, 1.5])


def calculate_average(list_values):
    try:
        return sum(list_values) / len(list_values)
    except ZeroDivisionError:
        return None


def no_collisions_occured(analyze_bag, start_time, end_time):
    try:
        collisions = analyze_bag.get_collisions(start_time, end_time)
    except:
        raise
        collisions = []
    print("\n")
    if len(collisions):
        t_last = -1.0
        for t in collisions:
            if(t - t_last >= 1):
                print("Collision at time %.3f!" % t)
                t_last = t
        print("One or more collisions occurred - no points are awarded!")
        return False
    else:
        print("No collisions occurred - good job!")
        return True


def print_scoring(average_value, max_value, value_type, unit, scores):
    if average_value is not None:
        score = get_score(average_value, max_value, scores)
        print("Average %s: %.3f %s" % (value_type, average_value, unit))
        print("Score for %s: %.2f" % (value_type, score))
    else:
        print("No %s evaluated, hence no scoring is provided." % value_type)
