/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <iomanip>
#include <iostream>
#include <map>
#include <vector>
#include "cli/options.hpp"

#include "performance_test/ros2/events_logger.hpp"
#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/utils.hpp"

namespace performance_test {

struct NamedExecutor {
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr executor;
    std::string name;
};

class System {
   public:
    System() = default;

    void add_node(std::vector<std::shared_ptr<Node>> nodes);

    void add_node(std::shared_ptr<Node> node);

    void spin(int duration_sec, bool wait_for_discovery = true,
              bool name_threads = true);

    void enable_events_logger(std::string events_logger_path);

    void save_latency_all_stats(
        const std::map<std::string, std::vector<double>>& system_latency_stats,
        std::string filename) const;

    void save_profiling_inter_node_stats(const std::string& filename) const;

    void save_profiling_first_to_last_node_stats(
        const std::map<std::string, std::vector<double>>& system_latency_stats,
        const std::string& filename) const;

    void print_latency_all_stats(
        const std::map<std::string, std::vector<double>>& system_latency_stats)
        const;

    typedef std::vector<std::pair<std::string, performance_test::Tracker>>
        Trackers;
    std::map<std::string, std::vector<double>>
        calc_first_to_last_node_latency_stats() const;
    std::map<std::string, std::vector<double>> sum_up_internode_latencies()
        const;

    std::shared_ptr<Trackers> get_trackers_of_node_system() const;

    void dump_summed_up_internode_latencies(
        const std::map<std::string, std::vector<double>>& latencies,
        const std::string& filename) const;

    void dump_absolute_profiling_timestamps(
        const std::string& file_basename) const;
    void dump_tracking_numbers(const std::string& file_basename) const;

    // for testing purposes
    std::map<std::string, std::vector<double>> sum_up_internode_latencies(
        const std::shared_ptr<Trackers> trackers_node_system) const;
    std::map<std::string, std::vector<double>>
        calc_first_to_last_node_latency_stats(
            const std::shared_ptr<Trackers> all_trackers) const;

   private:
    struct LoggingParameters {
        char separator = ',';
        int wide_space = 40;
        int narrow_space = 20;
    } _logging_parameters;

    void wait_discovery();

    void wait_pdp_discovery(
        std::chrono::milliseconds period = std::chrono::milliseconds(20),
        std::chrono::milliseconds max_pdp_time =
            std::chrono::milliseconds(30 * 1000));

    void wait_edp_discovery(
        std::chrono::milliseconds period = std::chrono::milliseconds(20),
        std::chrono::milliseconds max_edp_time =
            std::chrono::milliseconds(30 * 1000));

    void log_latency_all_stats(
        const std::map<std::string, std::vector<double>>& system_latency_stats,
        std::ostream& stream) const;

    void log_profiling_inter_node_stats(std::ostream& stream) const;
    void log_profiling_first_to_last_node_stats(
        const std::map<std::string, std::vector<double>>& system_latency_stats,
        std::ostream& stream) const;
    void log_latency_stats_header(std::ostream& stream) const;

    template<typename T>
    void dump_vector_data(const std::vector<T>& data,
                          std::ostream& stream) const {
        for (int i = 0; i < data.size(); i++) stream << data[i] << ",";

        if (data.size() > 0) stream << data.back();
        stream << std::endl;
    }

    bool is_corrupted_iteration(const std::vector<double>& samples) const;
    int get_minimum_iterations(
        const std::shared_ptr<Trackers> all_trackers) const;

    double mean(const std::vector<double>& samples) const;
    double stddev(const std::vector<double>& samples) const;

    std::chrono::high_resolution_clock::time_point _start_time;

    int _experiment_duration_sec;

    std::vector<std::shared_ptr<Node>> _nodes;

    std::map<int, NamedExecutor> _executors_map;

    std::shared_ptr<EventsLogger> _events_logger;

    // the following values are used for comparing different plots using the
    // python scripts
    bool _got_system_info;
    int _pubs;
    int _subs;
    float _frequency;
    size_t _msg_size;

    std::vector<std::string> _profiling_labels = get_profiling_labels();
};
}  // namespace performance_test