from typing import Dict, Any, Tuple, List
import json
import argparse
import os
import csv
from glob import glob
import dpath
from dpath.util import get
import shutil
import sys
import csv
from datetime import datetime
import logging

import numpy as np
LATENCY_LABELS = [ "e2e",
            "rclcpp_interprocess_publish",
            "rcl_publish",
            "rmw_publish",
            "dds_write",
            "sub_dds_on_data",
            "sub_rclcpp_take_enter",
            "sub_rcl_take_enter",
            "sub_rmw_take_enter",
            "sub_dds_take_enter",
            "sub_dds_take_leave",
            "sub_rmw_take_leave",
            "sub_rcl_take_leave",
            "sub_rclcpp_take_leave",
            "rclcpp_handle"]

PROFILING_CATEGORIES = ["overheadPubRclcpp",
            "overheadPubRmw",
            "overheadDds",
            "overheadSubRclcpp",
            "overheadSubRmw",
            "overheadUnlabelled",
            "overheadBenchmarking"]

SAMPLES_SKIP = 10
    
def get_reliability_threshold(meta_info: Dict[str, Any], median: float) -> Tuple[float, float]:
    #late_thresh = no_nodes * 0.2 / meta_info["experiment"]["publish_frequency"] * 1e6
    no_nodes = 3
    too_late_thresh = no_nodes / meta_info["experiment"]["publish_frequency"] * 1e6
    late_thresh = 0.3 * median
    return late_thresh, too_late_thresh

def get_current_timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d_%H-%M")

def is_profiling_topology(meta_info: Dict[str, Any]) -> bool:
    return meta_info["topology"]["filename"].find("profiled") > 0

def create_parameter_stamp(parameter_filter: Dict[str, Any], variable_parameter_val) -> str:
    name = "Fixed"
    for fixed_parameter in parameter_filter["fixed"].items():
        name += f"_{fixed_parameter[0].upper()}{fixed_parameter[1]}"
    name +="_"
    name += "Variable_" + parameter_filter["variable"]["parameter"].upper() + str(variable_parameter_val)
    return name

def create_csv_directory_name(parameter_filter: Dict[str, Any], comment: str) -> str:
    name = get_current_timestamp() + "_"
    name += create_parameter_stamp(parameter_filter, "ALL")
    name += comment
    return name

def read_meta_information(experiment_dir: str) -> Dict[str, Any]:
    with open(os.path.join(experiment_dir, "system_info.json")) as f:
        jsonData = json.load(f)
    return jsonData

def get_unreliable_samples(samples: np.array, meta_info: Dict[str, Any]) -> np.array:
    late_thresh, too_late_thresh = get_reliability_threshold(meta_info, np.median(samples))
    samples_too_late = np.where(samples > (np.median(samples) + late_thresh))[0]
    samples_too_early = np.where(samples < (np.median(samples) - late_thresh))[0]
    unreliable_samples = np.concatenate((samples_too_early, samples_too_late))

    return unreliable_samples

def calculate_statistics(results: Dict[str, Any], label: str, samples: np.array):
    results["median"][label].append(np.median(samples))
    results["mean"][label].append(np.mean(samples))
    results["std"][label].append(np.std(samples, ddof=1))
    results["lp"][label].append(np.percentile(samples, 25))
    results["hp"][label].append(np.percentile(samples, 75))
    results["lw"][label].append(np.min(samples))
    results["hw"][label].append(np.max(samples))

def get_node_list(experiment_dir: str) -> List[int]:
    files = glob(os.path.join(experiment_dir, "*header.csv"))
    nodes = []
    for f in files:
        filename = os.path.basename(f)
        nodes.append(int(filename[:filename.find("_")]) + 2)  # add two for first and last node
    nodes = sorted(nodes)
    return nodes

def read_csvs(experiment_dir: str) -> Tuple[Dict[str, Any], List[int]]:
    meta_info = read_meta_information(experiment_dir)
    results = {"mean": {key: [] for key in LATENCY_LABELS},
            "std": {key: [] for key in LATENCY_LABELS},
            "median": {key: [] for key in LATENCY_LABELS},
            "lp": {key: [] for key in LATENCY_LABELS},
            "hp": {key: [] for key in LATENCY_LABELS},
            "lw": {key: [] for key in LATENCY_LABELS},
            "hw": {key: [] for key in LATENCY_LABELS}}
    for key in results.keys():
        results[key].update({category: [] for category in PROFILING_CATEGORIES})
    nodes = get_node_list(experiment_dir)
    for n in nodes:
        msg_sent_timestamps, msg_rcvd_timestamps, absolute_profiling_timestamps = (
            read_dumped_timestamps(experiment_dir, n)
        )
      
        time_differences = calculate_time_differences_in_dpp(absolute_profiling_timestamps,
                                                            msg_sent_timestamps,
                                                            msg_rcvd_timestamps)
        process_time_differences(results, n, meta_info, time_differences)

    return results, nodes


def read_absolute_timestamps(experiment_dir: str, n: int, substr: str) -> List[np.array]:
    absolute_timestamps = [None for node_iter in range(n-1)]
    node_count = 0
    files = glob(os.path.join(experiment_dir, f"{n-2}_*{substr}*"))

    for f in files:
        with open(os.path.abspath(f)) as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            next(reader)
            node_count = 0
            for row in reader:
                absolute_timestamps[node_count] = np.array(row[1:], dtype=np.uint64)
                node_count += 1
    return absolute_timestamps

def read_dumped_timestamps(experiment_dir: str, n: int) -> Tuple[List[np.array], List[np.array], List[np.array]]:
    absolute_profiling_timestamps = {key: [None for node_iter in range(n-1)] for key in LATENCY_LABELS}
    for latency_label in LATENCY_LABELS:
        absolute_profiling_timestamps[latency_label] = read_absolute_timestamps(experiment_dir, n, latency_label)

    msg_sent_timestamps = read_absolute_timestamps(experiment_dir, n, "header.csv")
    msg_rcvd_timestamps = read_absolute_timestamps(experiment_dir, n, "now.csv")
    return msg_sent_timestamps, msg_rcvd_timestamps, absolute_profiling_timestamps

def get_minimum_amount_of_samples(msg_sent_timestamps: List[np.array]) -> int:
    min_samples = msg_sent_timestamps[0].size

    for tracker_timestamps in msg_sent_timestamps:
        min_samples = min(min_samples, tracker_timestamps.size)

    return min_samples

def reduce_to_minimum_amount_of_samples(min_samples: int,
                                        absolute_profiling_timestamps: Dict[str, List[np.array]],
                                        msg_sent_timestamps: List[np.array],
                                        msg_rcvd_timestamps: List[np.array]):
    for tracker_idx, tracker_timestamps in enumerate(msg_sent_timestamps):
        msg_sent_timestamps[tracker_idx] = tracker_timestamps[:min_samples]

    for tracker_idx, tracker_timestamps in enumerate(msg_rcvd_timestamps):
        msg_rcvd_timestamps[tracker_idx] = tracker_timestamps[:min_samples]

    for key in absolute_profiling_timestamps.keys():
        if not (key == "e2e"):
            for tracker_idx, tracker_timestamps in enumerate(absolute_profiling_timestamps[key]):
                absolute_profiling_timestamps[key][tracker_idx] = tracker_timestamps[:min_samples]

def calculate_time_differences_in_dpp(absolute_profiling_timestamps: Dict[str, List[np.array]],
                                      msg_sent_timestamps: List[np.array],
                                      msg_rcvd_timestamps: List[np.array]) -> Dict[str, np.array]:
    mininmum_amount_of_samples = get_minimum_amount_of_samples(msg_sent_timestamps)
    reduce_to_minimum_amount_of_samples(mininmum_amount_of_samples,
                                        absolute_profiling_timestamps,
                                        msg_sent_timestamps,
                                        msg_rcvd_timestamps)

    no_trackers = len(msg_rcvd_timestamps)
    no_samples = msg_rcvd_timestamps[0].size

    time_differences = {latency_label:np.zeros(no_samples) for latency_label in LATENCY_LABELS}

    for tracker_iter in range(no_trackers):
        for latency_label_idx, latency_label in enumerate(LATENCY_LABELS):
            if latency_label == "e2e":
                time_differences[latency_label] += (
                    (msg_rcvd_timestamps[tracker_iter] - msg_sent_timestamps[tracker_iter])
                    / 1e3
                )
            elif latency_label == "rclcpp_interprocess_publish":
                time_differences[latency_label] += (
                    (absolute_profiling_timestamps[latency_label][tracker_iter]
                    - msg_sent_timestamps[tracker_iter])
                    / 1e3
                )
            else:
                time_differences[latency_label] += (
                    (absolute_profiling_timestamps[latency_label][tracker_iter]
                    - absolute_profiling_timestamps[LATENCY_LABELS[latency_label_idx - 1]][tracker_iter])
                    / 1e3
                )
    return time_differences

def calculate_overhead_categories(results: Dict[str, Any],
                                  time_differences: Dict[str, Any]):
    calculate_statistics(results,
                            "overheadPubRclcpp",
                            time_differences["rmw_publish"] + time_differences["rcl_publish"] + time_differences["rclcpp_interprocess_publish"])
    calculate_statistics(results,
                        "overheadPubRmw",
                            time_differences["dds_write"])
    calculate_statistics(results,
                        "overheadDds",
                            time_differences["sub_dds_on_data"] + time_differences["sub_dds_take_leave"])
    calculate_statistics(results,
                            "overheadSubRclcpp",
                            time_differences["rclcpp_handle"] + time_differences["sub_rcl_take_enter"] + time_differences["sub_rclcpp_take_leave"] + time_differences["sub_rcl_take_leave"] + time_differences["sub_rmw_take_enter"])
    calculate_statistics(results,
                            "overheadSubRmw",
                            time_differences["sub_rmw_take_leave"] + time_differences["sub_dds_take_enter"])
    calculate_statistics(results,
                            "overheadUnlabelled",
                            time_differences["sub_rclcpp_take_enter"])

    # calculate overhead entailed by benchmarking
    time_differences_samples_overhead_benchmarking = np.zeros(time_differences["e2e"].size)
    for key in time_differences.keys():
        if not (key == "e2e"):
            time_differences_samples_overhead_benchmarking += time_differences[key]

    time_differences_samples_overhead_benchmarking = (
        time_differences["e2e"] - time_differences_samples_overhead_benchmarking
    )
    calculate_statistics(results,
                            "overheadBenchmarking",
                            time_differences_samples_overhead_benchmarking)

def process_time_differences(results: Dict[str, Any],
                             n: int,
                             meta_info: Dict[str, Any],
                             time_differences: Dict[str, np.array]) -> Dict[str, Any]:

    #unreliable_samples = get_unreliable_samples(time_differences["e2e"], meta_info)
    unreliable_samples = np.array([], dtype=int)
    for key in time_differences.keys():
        time_differences[key] = np.delete(time_differences[key], unreliable_samples)
        if len(time_differences[key]) <= 2:
            time_differences[key] = np.array([0, 0])
        else:
            calculate_statistics(results, key, time_differences[key])

    calculate_overhead_categories(results, time_differences)
    return results

def contains_system_info(experiment_dir: str) -> bool:
    return os.path.exists(os.path.join(experiment_dir, "system_info.json"))

def get_filter_glob(parameter_filter: Dict[str, Any], variable_parameter_val: Any) -> str:
    glob_string = ""
    if "publisher_frequency" in parameter_filter["fixed"].keys():
        glob_string += "*PublishFrequency" + str(parameter_filter["fixed"]["publisher_frequency"]) + "Hz"
    elif "publisher_frequency" == parameter_filter["variable"]["parameter"]:
        glob_string += "*PublishFrequency" + str(variable_parameter_val) + "Hz"

    if "payload" in parameter_filter["fixed"].keys():
        glob_string += "*MsgSize" + parameter_filter["fixed"]["payload"]
    elif "payload" == parameter_filter["variable"]["parameter"]:
        glob_string += "*MsgSize" + str(variable_parameter_val)

    if "middleware" in parameter_filter["fixed"].keys():
        glob_string += "*" + parameter_filter["fixed"]["middleware"]
    elif "middleware" == parameter_filter["variable"]["parameter"]:
        glob_string += "*" + str(variable_parameter_val)

    if "qos" in parameter_filter["fixed"].keys():
        glob_string += "*" + parameter_filter["fixed"]["qos"]
    elif "qos" == parameter_filter["variable"]["parameter"]:
        glob_string += "*" + str(variable_parameter_val)
    return glob_string

def save_latency_stats_to_csv_file(results: Dict[str, Any], nodes: List[str], parent_dir: str, file_annotation: str):
    FIELDNAMES = [fieldname for fieldname in results.keys()]
    FIELDNAMES.extend(["nodes"])

    data_dict = {}
    csv_filepath = os.path.join(
        parent_dir,
        os.path.basename(parent_dir) + file_annotation + ".csv"
    )
    if not os.path.exists(parent_dir):
        os.makedirs(parent_dir)
    with open(csv_filepath, 'w') as f:
        csvwriter = csv.DictWriter(f, delimiter=',', fieldnames=FIELDNAMES)
        csvwriter.writeheader()
        for node_idx, node in enumerate(nodes):
            data_dict.update({key: results[key][node_idx] for key in results.keys()})
            data_dict.update({"nodes": node})
            csvwriter.writerow(data_dict)

# taken from https://stackoverflow.com/questions/4869189/how-to-transpose-a-dataset-in-a-csv-file
def save_transposed_csv(csv_src_path: str, csv_dest_path: str):
    with open(csv_src_path) as f_src, open(csv_dest_path, 'w') as f_dest:
        csv_reader = csv.reader(f_src, delimiter=',')
        next(csv_reader)
        csv.writer(f_dest, delimiter=',').writerows(zip(*(row for row in csv_reader)))

def create_dumped_samples_csv_name(parameter_stamp: str, original_basename: str) -> str:
    substr = "_dpp_nodes"
    ind = original_basename.find(substr)
    return parameter_stamp + "_" + original_basename[:ind + len(substr)]

def update_latency_vs_x_stats(results_unsorted: Dict[str, Any],
                              results_sorted: Dict[str, Any],
                              nodes: List[str],
                              x_name: str,
                              x_val: int,
                              no_nodes=3) -> Dict[str, Any]:
    # find index where we have 0 pt node, i.e. 3 nodes
    # at first nodes
    results_sorted[x_name].append(x_val)
    for latency_label in (LATENCY_LABELS + PROFILING_CATEGORIES):
        node_index = nodes.index(no_nodes)
        
        for key in results_unsorted.keys():
            results_sorted[key][latency_label].append(results_unsorted[key][latency_label][node_index])
    return results_sorted

def generate_e2e_boxplot_for_paper_str(results: Dict[str, Any], nodes: List[int]) -> str:
    result_str = ""
    for node_index, node in enumerate(nodes):
        result_str += "boxplot prepared={\n"
        result_str += "lower whisker=" + str(results["lw"]["e2e"][node_index]) + ",\n"
        result_str += "lower quartile=" + str(results["lp"]["e2e"][node_index]) + ",\n"
        result_str += "median=" + str(results["median"]["e2e"][node_index]) + ",\n"
        result_str += "upper quartile=" + str(results["hp"]["e2e"][node_index]) + ",\n"
        result_str += "upper whisker=" + str(results["hw"]["e2e"][node_index]) + "}\n"
    return result_str

def save_latency_vs_x_results(results: Dict[str, Any], parameter_filter: Dict[str, Any], csv_dir_path: str):
    for statistical_quantity in ["mean", "std", "median", "lp", "hp", "lw", "hw"]:
        parameter_stamp = create_parameter_stamp(parameter_filter, -1)
        csvfile_name = parameter_stamp + "_LatencyVs" + parameter_filter["variable"]["parameter"] + '_' + statistical_quantity + '.csv'
        with open(os.path.join(csv_dir_path, csvfile_name), 'w') as f:
            stats = results[statistical_quantity]

            fieldnames = [parameter_filter["variable"]["parameter"]] + LATENCY_LABELS + PROFILING_CATEGORIES + ["too_late", "rcvd", "lost", "late"]
            writer = csv.DictWriter(f, fieldnames=fieldnames, delimiter=',')
            writer.writeheader()

            data_dict = {}
            for idx, x_val in enumerate(results[parameter_filter["variable"]["parameter"]]):
                data_dict.update({parameter_filter["variable"]["parameter"]: x_val})
                data_dict.update({label: stats[label][idx] for label in LATENCY_LABELS})
                data_dict.update({category: stats[category][idx] for category in PROFILING_CATEGORIES})
                writer.writerow(data_dict)

        print(f"Saved to {csv_dir_path}")

parser = argparse.ArgumentParser()
parser.add_argument("--results_dir", help="Parent directory of experiment directories", default=".")
parser.add_argument("--parameter_filter", default="*")
parser.add_argument("--paper_results_dir", default="paper_csvs")
parser.add_argument("--comment", default="")
args = parser.parse_args()

parameter_filter = json.loads(args.parameter_filter)

# craete directories
experiment_parent_dir = os.path.abspath(args.results_dir)
paper_results_dir = os.path.join(os.path.abspath(experiment_parent_dir), args.paper_results_dir)
if not os.path.exists(paper_results_dir):
    os.makedirs(paper_results_dir)

csv_dir_path = os.path.join(paper_results_dir, create_csv_directory_name(parameter_filter, args.comment))
if not os.path.exists(csv_dir_path):
    os.makedirs(csv_dir_path)

logging.basicConfig(filename=os.path.join(csv_dir_path, "log.txt"), level=logging.DEBUG)

# pre allocate variables
results_latency_vs_x = {"mean": {label: [] for label in LATENCY_LABELS},
                   "std": {label: [] for label in LATENCY_LABELS},
                   "median": {label: [] for label in LATENCY_LABELS},
                   "lp": {label: [] for label in LATENCY_LABELS},
                   "hp": {label: [] for label in LATENCY_LABELS},
                   "lw": {label: [] for label in LATENCY_LABELS},
                   "hw": {label: [] for label in LATENCY_LABELS},
                   }
for key in results_latency_vs_x.keys():
    results_latency_vs_x[key].update({profiling_category: [] for profiling_category in PROFILING_CATEGORIES})
results_latency_vs_x.update({parameter_filter["variable"]["parameter"]: []})

results_latency = {}
nodes = []
print("bla")
# iterate over all "x" values
for variable_parameter_val in parameter_filter["variable"]["values"]:
    # get current glob
    filter_glob = get_filter_glob(parameter_filter, variable_parameter_val)
    
    print(f"Current parameter value is {variable_parameter_val}.")
    print(f"The following filter is being used: {filter_glob} ")
    print(glob(os.path.join(experiment_parent_dir, filter_glob)))
    print(os.path.join(experiment_parent_dir, filter_glob))
    # go through the found directories
    for experiment_dir in glob(os.path.join(experiment_parent_dir, filter_glob)):
        # create a unique parameter stamp identifying our current scenario
        parameter_stamp = create_parameter_stamp(parameter_filter,
                                                    variable_parameter_val)

        print(f"Processing {experiment_dir}")
        results_latency, nodes = read_csvs(os.path.abspath(experiment_dir))

        # save calculated statistics stuff in csv file
        csv_parent_dir = os.path.join(csv_dir_path, parameter_stamp)
        for key in results_latency.keys():
            save_latency_stats_to_csv_file(results_latency[key], nodes, csv_parent_dir, "_dumped_" + str(key))

        shutil.copy(
            os.path.join(experiment_dir, "system_info.json"),
            os.path.join(csv_parent_dir, parameter_stamp + ".json")
        )

        # copy dumped values files to post-processed results folder
        # and transpose the content
        dumped_values_files = glob(
            os.path.join(experiment_dir, "*_summed_up_internode_latencies*")
        )
        for dumped_values_file in dumped_values_files:
            if not os.path.exists(os.path.join(csv_parent_dir, "dumped_samples")):
                os.makedirs(os.path.join(csv_parent_dir, "dumped_samples"))

            dumped_samples_csv_name = create_dumped_samples_csv_name(
                parameter_stamp,
                os.path.basename(dumped_values_file))
            save_transposed_csv(
                dumped_values_file,
                os.path.join(csv_parent_dir, "dumped_samples",
                            dumped_samples_csv_name + ".csv"))

        # update latency_vs_x stats
        results_latency_vs_x = update_latency_vs_x_stats(results_latency,
                                                        results_latency_vs_x,
                                                        nodes,
                                                        parameter_filter["variable"]["parameter"],
                                                        variable_parameter_val)

    parameter_stamp = create_parameter_stamp(parameter_filter, -1)


    # create for latency vs nodes the boxplot that we can copy pgfplot compliant
    with open(os.path.join(csv_dir_path, parameter_stamp + "_" + str(variable_parameter_val) + "_boxplot_for_paper.txt"), 'w') as f:
        result_str = generate_e2e_boxplot_for_paper_str(results_latency, nodes)
        f.write(result_str)

save_latency_vs_x_results(results_latency_vs_x, parameter_filter, csv_dir_path)
# save execution stats
with open(os.path.join(csv_dir_path, "parameters.json"), 'w') as f:
    json.dump(parameter_filter, f, indent=4)