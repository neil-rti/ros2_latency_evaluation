/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <limits.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "performance_test/ros2/communication.hpp"
#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/resource_usage_logger.hpp"
#include "performance_test/ros2/system.hpp"

#include "performance_test_factory/factory.hpp"

#include "cli/options.hpp"
#include "utils.cpp"

using namespace nlohmann;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
    auto options = benchmark::Options(argc, argv);

    auto json_list = options.topology_json_list;

    print_user_info(options, json_list);

    std::string topology_path = json_list[0];
    pid_t pid = getpid();

    // Load topology from json file
    performance_test::TemplateFactory factory =
        performance_test::TemplateFactory(options.ipc, options.ros_params);
    json topology_j = factory.read_json_file(topology_path);

    std::vector<std::string> payloads;

    if (options.msg_size != "")
        payloads.push_back("stamped" + options.msg_size);
    else
        std::vector<std::string> payloads =
            factory.read_payloads_from_file(options.payload_file);

    // change middleware
    set_middleware(options.middle_ware);

    // create results directory and strings
    // todo: multiple processes/ topologies
    std::string results_dir_name = create_results_dir_name(options);
    std::string make_dir = "mkdir -p " + results_dir_name;
    std::string experiment_tag;

    std::string resources_output_path;
    std::string events_output_path;
    std::string latency_all_output_path;
    std::string profiling_inter_node_output_path;
    std::string profiling_first_to_last_node_output_path;
    std::string first_to_last_node_latencies_pgf_csv;
    std::string summed_up_internode_latencies_output_path;
    std::string dumped_profile_timestamps_output_base_path;
    std::string dumped_tracking_numbers_ouput_base_path;

    const auto ret = system(make_dir.c_str());
    static_cast<void>(ret);

    // create csv files here that store the results of the whole walkthrough
    // required for plotting the results in the paper later on

    std::ofstream nodes_vs_first_to_last_node_latencies_pgf_csv_stream(
        results_dir_name +
        "/nodes_vs_first_to_last_node_latencies_walkthrough.csv");

    create_nodes_vs_first_to_last_node_latencies_haeder(
        nodes_vs_first_to_last_node_latencies_pgf_csv_stream);

    for (int i = options.data_processing_pipeline_range_start; i <= options.data_processing_pipeline_range_end;
         i += options.data_processing_pipeline_range_step_size) {
        std::cout << "Evaluating " << i << " nodes in data-processing pipeline of "
                  << options.data_processing_pipeline_range_end << " in total." << std::endl;

        for (std::string payload : payloads) {
            if (payload != "")
                std::cout << "Evluating the following payload: " << payload
                          << std::endl;
            else
                payload = topology_j["nodes"][0]["publishers"][0]["msg_type"];

            if (options.data_processing_pipeline_range_start == options.data_processing_pipeline_range_end)
                experiment_tag = "";
            else
                experiment_tag = std::to_string(i) + "_dpp_";

            if (payload != "") experiment_tag += "_" + payload + "_";

            resources_output_path =
                results_dir_name + "/" + experiment_tag + "resources.txt";
            events_output_path =
                results_dir_name + "/" + experiment_tag + "events.txt";
            latency_all_output_path =
                results_dir_name + "/" + experiment_tag + "latency_all.csv";
            profiling_inter_node_output_path = results_dir_name + "/" +
                                               experiment_tag +
                                               "profiling_inter_node.csv";
            profiling_first_to_last_node_output_path =
                results_dir_name + "/" + experiment_tag +
                "profiling_first_to_last_node.csv";
            summed_up_internode_latencies_output_path =
                results_dir_name + "/" + experiment_tag +
                "summed_up_internode_latencies.csv";
            dumped_profile_timestamps_output_base_path =
                results_dir_name + "/" + experiment_tag +
                "absolute_profiling_timestamps";
            dumped_tracking_numbers_ouput_base_path =
                results_dir_name + "/" + experiment_tag + "tracking_numbers";

            rclcpp::init(argc, argv);
            // Start resources logger
            performance_test::ResourceUsageLogger ru_logger(
                resources_output_path);
            ru_logger.start(
                std::chrono::milliseconds(options.resources_sampling_per_ms));

            performance_test::System ros2_system;

            if (options.tracking_options.is_enabled) {
                ros2_system.enable_events_logger(events_output_path);
            }

            factory.change_data_processing_pipeline(topology_j, i);
            factory.change_payload_size(payload, topology_j);
            factory.change_publish_period_ms(
                static_cast<int>(1e3 / options.publish_frequency), topology_j);
            factory.change_qos_policy_value(
                "qos_reliability", options.qos_reliability, topology_j);
            auto nodes_vec = factory.parse_json_to_node_system(
                topology_j, options.tracking_options);

            ros2_system.add_node(nodes_vec);

            // now the system is complete and we can make it spin for the
            // requested duration
            bool wait_for_discovery = true;
            ros2_system.spin(options.duration_sec, wait_for_discovery);

            // terminate the experiment
            ru_logger.stop();
            rclcpp::shutdown();
            std::this_thread::sleep_for(1000ms);

            std::map<std::string, std::vector<double>>
                first_to_last_node_latency_stats =
                    ros2_system.calc_first_to_last_node_latency_stats();
            std::map<std::string, std::vector<double>>
                summed_up_internode_latencies =
                    ros2_system.sum_up_internode_latencies();

            ros2_system.print_latency_all_stats(
                first_to_last_node_latency_stats);
            ros2_system.save_latency_all_stats(first_to_last_node_latency_stats,
                                               latency_all_output_path);
            ros2_system.save_profiling_inter_node_stats(
                profiling_inter_node_output_path);
            ros2_system.save_profiling_first_to_last_node_stats(
                first_to_last_node_latency_stats,
                profiling_first_to_last_node_output_path);

            ros2_system.dump_summed_up_internode_latencies(
                summed_up_internode_latencies,
                summed_up_internode_latencies_output_path);
            ros2_system.dump_absolute_profiling_timestamps(
                dumped_profile_timestamps_output_base_path);
            ros2_system.dump_tracking_numbers(
                dumped_tracking_numbers_ouput_base_path);

            save_nodes_vs_first_to_last_node_latencies_to_pgf_csv(
                nodes_vec.size(), first_to_last_node_latency_stats,
                nodes_vs_first_to_last_node_latencies_pgf_csv_stream);

            // Parent process: wait for children to exit and print system stats
            if (pid != 0) {
                if (json_list.size() > 1) {
                    waitpid(getpid() + 1, &pid, 0);
                }
            }
        }
        save_system_info(topology_j, results_dir_name, "system_info.json",
                         options);
    }
}
