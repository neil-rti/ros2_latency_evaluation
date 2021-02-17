/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include "performance_test/ros2/tracker.hpp"

#include "cxxopts.hpp"

namespace benchmark {

class Options {
   public:
    Options() {
        ipc = true;
        ros_params = true;
        name_threads = true;
        duration_sec = 5;
        resources_sampling_per_ms = 500;
        tracking_options.is_enabled = true;
        tracking_options.late_percentage = 20;
        tracking_options.late_absolute_us = 5000;
        tracking_options.too_late_percentage = 100;
        tracking_options.too_late_absolute_us = 50000;
        tracking_options.enable_profiling = false;
        experiment_name = "experiment";
        data_processing_pipeline_range_start = 1;
        data_processing_pipeline_range_end = 1;
        data_processing_pipeline_range_step_size                 = 1;
        payload_file                          = "";
        middle_ware                           = "";
        publish_frequency                     = 0;
        msg_size                              = "";
        qos_reliability                       = "";
    };

    Options(int argc, char** argv)
        : Options()
    {
        parse(argc, argv);
    }

    void parse(int argc, char** argv)
    {
      // Parse arguments
      cxxopts::Options options(argv[0], "ROS2 performance benchmark");
      std::string ipc_option;
      std::string ros_params_option;
      std::string name_threads_option;
      std::string tracking_enabled_option;
      std::string profiling_option;
      options.positional_help("FILE [FILE...]").show_positional_help();
      options.parse_positional({"topology"});
      options.add_options()
      ("h,help", "print help")
      ("topology", "json file(s) describing the topology of the system",
        cxxopts::value<std::vector<std::string>>(topology_json_list),"FILE [FILE...]")
      ("ipc", "intra-process-communication",
        cxxopts::value<std::string>(ipc_option)->default_value(ipc ? "on" : "off"),"on/off")
      ("ros_params", "enable parameter services",
        cxxopts::value<std::string>(ros_params_option)->default_value(ros_params ? "on" : "off"),"on/off")
      ("name_threads", "enable naming threads",
        cxxopts::value<std::string>(name_threads_option)->default_value(name_threads ? "on" : "off"),"on/off")
      ("t,time", "test duration", cxxopts::value<int>(duration_sec)->default_value(std::to_string(duration_sec)),"sec")
      ("s, sampling", "resources sampling period",
        cxxopts::value<int>(resources_sampling_per_ms)->default_value(std::to_string(resources_sampling_per_ms)),"msec")
      ("tracking", "compute and logs detailed statistics and events",
        cxxopts::value<std::string>(tracking_enabled_option)->default_value(tracking_options.is_enabled ? "on" : "off"),"on/off")
      ("late-percentage", "a msg with greater latency than this percentage of the msg publishing period is considered late",
        cxxopts::value<int>(tracking_options.late_percentage)->default_value(std::to_string(tracking_options.late_percentage)),"%")
      ("late-absolute", "a msg with greater latency than this is considered late",
        cxxopts::value<int>(tracking_options.late_absolute_us)->default_value(std::to_string(tracking_options.late_absolute_us)),"usec")
      ("too-late-percentage", "a msg with greater latency than this percentage of the msg publishing period is considered lost",
        cxxopts::value<int>(tracking_options.too_late_percentage)->default_value(std::to_string(tracking_options.too_late_percentage)),"%")
      ("too-late-absolute", "a msg with greater latency than this is considered lost",
        cxxopts::value<int>(tracking_options.too_late_absolute_us)->default_value(std::to_string(tracking_options.too_late_absolute_us)),"usec")
      ("data-processing-pipeline-range-start", "range that the number of nodes in the data-process pipeline should start with",
        cxxopts::value<int>(data_processing_pipeline_range_start))
      ("data-processing-pipeline-range-end", "range that the number of nodes in the data-processing pipeline should end with",
        cxxopts::value<int>(data_processing_pipeline_range_end))
      ("data-processing-pipeline-range-step-size", "step size of range of the number of nodes in the data-processing pipeline",
        cxxopts::value<int>(data_processing_pipeline_range_step_size))
      ("experiment-name", "name of the experiment",
        cxxopts::value<std::string>(experiment_name)->default_value("experiment"))
      ("payloads", "Relative path to file that contains payloads to be used",
        cxxopts::value<std::string>(payload_file))
      ("msg_size", "Msg size to be used, available ones are 100b, 1kb, 10kb, 500kb",
        cxxopts::value<std::string>(msg_size)->default_value("1kb"))
      ("profiling", "enable profiling (note msg size and IPC)",
        cxxopts::value<std::string>(profiling_option)->default_value(tracking_options.enable_profiling ? "on" : "off"),"on/off")
      ("middleware", "dds middleware to be used",
        cxxopts::value<std::string>(middle_ware)->default_value("fastrtps"))
      ("publish_frequency", "frequency with which messages are published",
        cxxopts::value<int>(publish_frequency))
      ("qos_reliability", "define qos reliability",
        cxxopts::value<std::string>(qos_reliability)->default_value("best-effort"));

      try {
        auto result = options.parse(argc, argv);

        if (result.count("help")) {
          std::cout << options.help() << std::endl;
          exit(0);
        }

        if (result.count("topology") == 0) {
          std::cout << "Please specify a json topology file" << std::endl;
          exit(1);
        }

        if (ipc_option != "off" && ipc_option != "on") {
          throw cxxopts::argument_incorrect_type(ipc_option);
        }

        if (tracking_enabled_option != "off" &&
            tracking_enabled_option != "on") {
          throw cxxopts::argument_incorrect_type(tracking_enabled_option);
        }

        if (data_processing_pipeline_range_start > data_processing_pipeline_range_end) {
          std::cout
              << "First value of data_processing_pipeline range must be smaller than second value"
              << std::endl;
          exit(1);
        }

        if (data_processing_pipeline_range_start == 0 || data_processing_pipeline_range_end == 0) {
          std::cout << "Values for the data_processing_pipeline range must be larger than zero."
                    << std::endl;
          exit(1);
        }

        if (publish_frequency < 0) {
          std::cout << "Publish_freuqncy must be positive integer!"
                    << std::endl;
                exit(1);
            }

            if (publish_frequency < 0) {
                std::cout << "Publish_freuqncy must be positive integer!"
                          << std::endl;
                exit(1);
            }

            if (std::find(SUPPORTED_MIDDLEWARES.begin(),
                          SUPPORTED_MIDDLEWARES.end(),
                          middle_ware) == SUPPORTED_MIDDLEWARES.end()) {
                std::cout << "This middleware version is not supported!"
                          << std::endl;
                exit(1);
            }

            if (std::find(SUPPORTED_MSG_SIZES.begin(),
                          SUPPORTED_MSG_SIZES.end(),
                          msg_size) == SUPPORTED_MSG_SIZES.end()) {
                std::cout << "This msg size is not supported!" << std::endl;
                exit(1);
            }
            if (std::find(SUPPORTED_QOS_RELIABILITIES.begin(),
                          SUPPORTED_QOS_RELIABILITIES.end(), qos_reliability) ==
                SUPPORTED_QOS_RELIABILITIES.end()) {
                std::cout << "This qos reliability setting is not supported!"
                          << std::endl;
                exit(1);
            }
        } catch (const cxxopts::OptionException& e) {
            std::cout << "Error parsing options. " << e.what() << std::endl;
            exit(1);
        }

        ipc = (ipc_option == "on" ? true : false);
        ros_params = (ros_params_option == "on" ? true : false);
        name_threads = (name_threads_option == "on" ? true : false);
        tracking_options.is_enabled =
            (tracking_enabled_option == "on" ? true : false);
        tracking_options.enable_profiling =
            (profiling_option == "on" ? true : false);
    }

    bool ipc;
    bool ros_params;
    bool name_threads;
    int duration_sec;
    int resources_sampling_per_ms;
    std::vector<std::string> topology_json_list;
    performance_test::Tracker::TrackingOptions tracking_options;
    int data_processing_pipeline_range_start;
    int data_processing_pipeline_range_end;
    int data_processing_pipeline_range_step_size;
    std::string experiment_name;
    std::string payload_file;
    std::string middle_ware;
    int publish_frequency;
    std::string msg_size;
    std::string qos_reliability;
    const std::vector<std::string> SUPPORTED_MIDDLEWARES = {
        "cyclonedds", "connextdds", "fastrtps", "fastrtps_dynamic"};
    const std::vector<std::string> SUPPORTED_MSG_SIZES = {"100b", "1kb", "10kb",
                                                          "100kb", "500kb"};
    const std::vector<std::string> SUPPORTED_QOS_RELIABILITIES = {
        "reliable", "best-effort"};
};
}  // namespace benchmark
