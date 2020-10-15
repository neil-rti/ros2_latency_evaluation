 # Software License Agreement (BSD License)
 #
 #  Copyright (c) 2019, iRobot ROS
 #  All rights reserved.
 #
 #  This file is part of ros2-performance, which is released under BSD-3-Clause.
 #  You may use, distribute and modify this code under the BSD-3-Clause license.
 #

#!/usr/bin/env python

# Plot the aggregated results of multinode statistics collected during several experiments.
# Communication latency and reliability are plotted.
# For subscriber nodes latency is computed as the difference between a message timestamp
# and the time when it's received. For clients it's the time between when a request is issued and when a response
# is received.
# Reliability_sub is computed as the percentage of received messages/response wrt their total number.
# Reliability_pub is computed as the percentage of sent msgs/total msgs.
# Reliability_tot is the system reliability, computed as (rel_pub * rel_sub).
# Usage example:
# python3 ros_performance_plot.py <OUTPUT_DIRECTORY> --x subs --y reliability_sub --y2 latency

import argparse
import os
import sys

import matplotlib.pyplot as plt

import data_utils
import plot_common


def main(argv):

    parser = argparse.ArgumentParser()
    parser.add_argument('dir_paths', nargs='+', type=str, default="", help='path to the directory containing the scripts we want to plot')
    parser.add_argument('--x', type=str, required=True, choices=['pubs', 'subs', 'msg_size'], help='value to display on x axis')
    parser.add_argument('--y', type=str, nargs='+', required=True, choices=['latency', 'reliability_sub', 'reliability_pub', 'reliability_tot', 'max_frequency', 'msg_size'], help='value to display on y axis')
    parser.add_argument('--y2', type=str, nargs='+', default=[], choices=['latency', 'reliability_sub', 'reliability_pub', 'reliability_tot', 'max_frequency', 'msg_size'], help='value to display on an additional y axis')
    parser.add_argument('--separator', nargs='+', default=[], choices=['spin_frequency', 'send_frequency', 'msg_size', 'directory', 'duration', 'pubs', 'subs'], help='if not set all data are aggregated together, else aggregates only data which have the same value for the separator keys')
    parser.add_argument('--enable-profiling', type=bool, default=False)
    args = parser.parse_args()

    dir_paths = args.dir_paths
    x_axis = args.x
    y1_axis = args.y
    y2_axis = args.y2
    separator = args.separator

    __UNCOUNTABLE_DATA__ = ['directory', 'pubs', 'subs', 'spin_frequency', 'send_frequency', 'msg_size', 'separator']

    # Get all files in folders in alphabetic order
    latency_files = data_utils.get_files_from_paths(dir_paths, "latency_all.txt")
    profiling_files = data_utils.get_files_from_paths(dir_paths, "profiling_all.txt")

    latencies = []
    profiling_values = []
    # Collect data from csv files
    for file_path in latency_files:

        latency_csv_parsed = data_utils.parse_csv(file_path)
        
        if not latency_csv_parsed:
            continue

        latencies.append(latency_csv_parsed)

    for file_path in profiling_files:
        profiling_log_file_parsed = data_utils.parse_profiling_log_file(file_path)
        if not profiling_files:
            continue

        profiling_values.append(profiling_log_file_parsed)

    latency_data = plot_common.organize_data(latencies, x_axis, separator, __UNCOUNTABLE_DATA__)
    profiling_data = plot_common.organize_data(profiling_values, x_axis, separator, __UNCOUNTABLE_DATA__)
    ax, fig = plot_common.plot_function(latency_data, x_axis, y1_axis, y2_axis, separator, savefig=True)
    #breakpoint()
    plot_common.plot_profiling_over_subs(profiling_data, ax)
    plt.show()
    plt.savefig(os.path.join(dir_paths[0], 'latency_plot.pdf'))

if __name__ == '__main__':
    main(sys.argv[1:])

