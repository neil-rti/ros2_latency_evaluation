 # Software License Agreement (BSD License)
 #
 #  Copyright (c) 2019, iRobot ROS
 #  All rights reserved.
 #
 #  This file is part of ros2-performance, which is released under BSD-3-Clause.
 #  You may use, distribute and modify this code under the BSD-3-Clause license.
 #

#!/usr/bin/env python

# general utilities for files and data handling

import csv
import json
import os
import re
import sys

from io import StringIO

def get_files_from_paths(paths, file_substr: str = ""):
    '''returns a list of paths to files, given one or more directory or file paths.
        path files are sorted in ascending natural order
        
        file_substr (str): substring of file name that should be contained'''

    def atoi(text):
        return int(text) if text.isdigit() else text

    def natural_keys(text):
        '''
        alist.sort(key=natural_keys) sorts in human order
        http://nedbatchelder.com/blog/200712/human_sorting.html
        (See Toothy's implementation in the comments)
        '''
        return [ atoi(c) for c in re.split(r'(\d+)', text) ]

    file_paths_list = []
    for path in paths:
        if os.path.isdir(path):
            this_dir_files = os.listdir(path)
            this_dir_paths = [os.path.join(path, filename) for filename in this_dir_files if file_substr in filename]
            file_paths_list += this_dir_paths
        elif os.path.isfile(path):
            file_paths_list.append(path)
        else:
            print(path, " is not a valid file or directory!")
            sys.exit(1)

    file_paths_list.sort(key=natural_keys)

    return file_paths_list


def parse_target_json(target_file, experiment_type = ""):
    '''experiment_type can be "resources" or "latency_total"'''

    json_data = open(target_file)
    parsed_json = json.load(json_data)

    if experiment_type:
        return parsed_json[experiment_type]

    merged_target = {**parsed_json["resources"], **parsed_json["latency_total"]}

    return merged_target


def parse_csv_dict(file_path, delimiter='\t'):
    '''parse a file converting it into a csv dictionary'''

    with open(file_path, 'r') as file:
        file_string = file.read()

    # the input may not be a proper csv so I have to convert it
    csv_file = re.sub('[ ]+', delimiter, file_string)
    f = StringIO(csv_file)

    reader = csv.DictReader(f, delimiter=delimiter)

    return reader


def merge_dictionaries(dict1, dict2, uncountable_data=[]):
    '''
    merge two dictionaries with the following logic
    dict1 = {'both1':1, 'both2':2, 'only_1': 100 }
    dict2 = {'both1':10, 'both2': 20, 'only_2':200 }
    ---->
    merged_dict = {'only_2': 200, 'both2': 22, 'both1': 11, 'only_1': 100}

    this method handles multiple datatypes, as long as they support the `+` operator

    the uncountable_data array is used to specify those fields which can't be summed together
    '''

    #TODO: throw an error if trying to mix dictionaries with different values of an uncountable data

    merged_dict = {}

    # insert all items from dict1 in the merged dictionary, checking also their presence in dict2
    for key, value in dict1.items():

        if key in uncountable_data:
            merged_dict[key] = value
        else:
            default_val = type(value)()
            merged_dict[key] = value + dict2.get(key, default_val)

    # insert all items from dict2 which are not present in dict1 in the merged dictionary
    for key, value in dict2.items():

        if key in uncountable_data:
            merged_dict[key] = value
        elif key not in dict1:
            merged_dict[key] = value


    return merged_dict


def depth(d, level=0):
    if not isinstance(d, dict) or not d:
        return level
    return max(depth(d[k], level + 1) for k in d)


def get_unit_of_measure(label):
    # NOTE: this assumes that unit of measures is within square brackets at the end of the key
    search_result = re.search('\[(.+)\]$', label)
    if search_result:
        return search_result.group(1)
    else:
        return ""


def remove_unit_of_measure(label):
    # NOTE: this assumes that unit of measures is within square brackets at the end of the key
    new_label = re.sub('\[(.+)\]$', '', label)
    return new_label



def parse_csv(file_path):
    '''parses a csv into a dictionary structure, given its filepath'''

    # get the basename of the directory where this csv is located
    dir_name = os.path.basename(os.path.dirname(file_path))

    # create an empty structure as starting point
    data = {
        'directory': dir_name,      # number of nodes with at least a publisher or a server
        'pubs': 0,                  # number of nodes with at least a publisher or a server
        'subs': 0,                  # number of nodes with at least a subscriber or a client
        'valid_subscribers' : 0,    # number of subscribers or clients with at least 1 msg or srv::response received
        'received_count' : 0,       # total number of msg or srv::response received
        'sent_count' : 0,           # total number of msg published or srv called  TODO: check how to store here srv called
        'th_count' : -1,            # theoretical number of msgs that should be published by the publisher
        'latency' : 0,              # total latency of msg or srv::response received
        'spin_frequency' : 0,       # subscribers or servers spin frequency
        'send_frequency' : 0,       # publishers or clients frequency
        'msg_size' : 0,             # type of exchanged messages
        'duration': 0,              # total duration of the experiment in seconds
        'throughput': 0
    }

    # helper variables needed to fill the dictionary
    pubs_ids = set()
    subs_ids = set()

    reader = parse_csv_dict(file_path)

    rows_number = 0
    for row_dict in reader:
        rows_number += 1
        # store the fields of the current csv row into variables, adjustind their data type
        node_name = row_dict['node']
        topic = row_dict['topic']
        msg_count = int(row_dict.get('received[#]', 0))
        spin_frequency = int(row_dict.get('spin_frequency', -1))
        avg_latency = float(row_dict.get('mean[us]', 0))
        send_frequency = int(row_dict.get('freq[hz]', 0))
        msg_size = int(row_dict.get('size[b]', 0))
        experiment_duration = int(row_dict.get('duration[s]', 0))
        throughput = float(row_dict.get('throughput_mean[b/s]', 0))

        data['msg_size'] = msg_size
        data['duration'] = experiment_duration
        pubs_ids.add(topic)

        if node_name == topic:
            # this row denotes a publisher or server
            data['sent_count'] += msg_count
            data['send_frequency'] = send_frequency
            data['th_count'] = send_frequency * experiment_duration
        else:
            # this row denotes a subscriber or client
            subs_ids.add(node_name)

            data['received_count'] += msg_count
            data['spin_frequency'] = spin_frequency
            if msg_count > 0:
                # this row denotes a valid subscriber or client (it has received at least 1 msg or srv::response)
                data['latency'] += avg_latency
                data['valid_subscribers'] += 1
                data['send_frequency'] = send_frequency
                data['th_count'] = send_frequency * experiment_duration
                data['throughput'] = throughput

    if rows_number < 1:
        return {}

    data['pubs'] = len(pubs_ids)
    data['subs'] = len(subs_ids)

    # choose the most reliable source for sent_count.
    # if publishers are in a different csv, data['sent_count'] is 0, but the subscriber can still get the theoretical count
    # NOTE: this estimate will be wrong if the publisher was not able to achieve its requested publish frequency
    if data['sent_count'] == 0:
        data['sent_count'] = data['th_count']

    return data

def parse_profiling_log_file(file_path):
    '''parses a csv into a dictionary structure, given its filepath'''

    # get the basename of the directory where this csv is located
    dir_name = os.path.basename(os.path.dirname(file_path))

    # create an empty structure as starting point
    data = {
        'directory': dir_name,      # number of nodes with at least a publisher or a server
        'pubs': 0,                  # number of nodes with at least a publisher or a server
        'subs': 0,                  # number of nodes with at least a subscriber or a client
        'valid_subscribers' : 0,    # number of subscribers or clients with at least 1 msg or srv::response received
        'latency': 0,  # total latency of msg or srv::response received

    }

    # helper variables needed to fill the dictionary
    pubs_ids = set()
    subs_ids = set()

    # TODO: is there a way to support multiple frequencies in the same csv? right now I check that they are all equal
    # TODO: the `received_count` variables should be divided per topic. The publisher which starts first will send more messages than the others.
    # moreover this will be required once there is support for different frequencies.
    # TODO: some values (spin and send frequencies, msg_size, duration) should be equal among all lines. The check has been removed now

    reader = parse_csv_dict(file_path)

    rows_number = 0
    for row_dict in reader:
        rows_number += 1
        # store the fields of the current csv row into variables, adjustind their data type
        node_name = row_dict['node']
        topic = row_dict['topic']
        data['rclcpp_interprocess_publish_mean'] = float(row_dict.get('rclcpp_interprocess_publish_mean[us]', 0))
        data['rcl_publish_mean'] = float(row_dict.get('rcl_publish_mean[us]', 0))
        data['rmw_publish_mean'] = float(row_dict.get('rmw_publish_mean[us]', 0))
        data['dds_write_mean'] = float(row_dict.get('dds_write_mean[us]', 0))
        data['sub_dds_on_data_mean'] = float(row_dict.get('sub_dds_on_data_mean[us]', 0))
        data['sub_rclcpp_take_mean'] = float(row_dict.get('sub_rclcpp_take_mean[us]', 0))
        data['sub_rcl_take_mean'] = float(row_dict.get('sub_rcl_take_mean[us]', 0))
        data['sub_rmw_take_with_info_mean'] = float(row_dict.get('sub_rmw_take_with_info_mean[us]', 0))
        data['sub_dds_take_mean'] = float(row_dict.get('sub_dds_take_mean[us]', 0))
        data['sub_rclcpp_handle_mean'] = float(row_dict.get('sub_rclcpp_handle_mean[us]', 0))
        data['total_mean'] = float(row_dict.get('total_mean[us]', 0))

        pubs_ids.add(topic)

        if node_name != topic:
            # this row denotes a subscriber or client
            subs_ids.add(node_name)

    if rows_number < 1:
        return {}

    data['pubs'] = len(pubs_ids)
    data['subs'] = len(subs_ids)

    return data
