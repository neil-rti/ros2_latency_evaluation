/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <pthread.h>
#include <fstream>
#include <iostream>
#include <map>

#include "performance_test/ros2/names_utilities.hpp"
#include "performance_test/ros2/system.hpp"

void performance_test::System::enable_events_logger(
    std::string events_logger_path) {
    _events_logger = std::make_shared<EventsLogger>(events_logger_path);
}

void performance_test::System::add_node(
    std::vector<std::shared_ptr<Node>> nodes) {
    for (auto node : nodes) {
        this->add_node(node);
    }
}

void performance_test::System::add_node(std::shared_ptr<Node> node) {
    if (_events_logger != nullptr) {
        node->set_events_logger(_events_logger);
    }

    int executor_id = node->get_executor_id();
    auto it = _executors_map.find(executor_id);
    if (it != _executors_map.end()) {
        auto& ex = it->second;
        ex.executor->add_node(node);
        ex.name = ex.name + "_" + node->get_name();
    } else {
        auto ex = NamedExecutor();
        ex.executor =
            std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
        ex.executor->add_node(node);
        ex.name = node->get_name();

        _executors_map.insert(std::make_pair(executor_id, ex));
    }

    _nodes.push_back(node);
}

void performance_test::System::spin(int duration_sec, bool wait_for_discovery,
                                    bool name_threads) {
    _experiment_duration_sec = duration_sec;
    // Store the instant when the experiment started
    _start_time = std::chrono::high_resolution_clock::now();

    // Check if some nodes have been added to this System
    if (_nodes.empty()) {
        assert(0 &&
               "Error. Calling performance_test::System::spin when no nodes "
               "have been "
               "added.");
    }

    if (_events_logger != nullptr) {
        _events_logger->set_start_time(_start_time);
    }

    if (wait_for_discovery) {
        // wait until PDP and EDP are finished before starting
        // log events when each is completed
        this->wait_discovery();
    }

    for (const auto& pair : _executors_map) {
        auto& name = pair.second.name;
        auto& executor = pair.second.executor;

        // Spin each executor in a separate thread
        std::thread thread([=]() { executor->spin(); });
        if (name_threads) {
            pthread_setname_np(thread.native_handle(), name.c_str());
        }
        thread.detach();
    }

    // let the nodes spin for the specified amount of time
    std::this_thread::sleep_for(std::chrono::seconds(_experiment_duration_sec));

    // after the timer, stop all the spin functions
    for (const auto& pair : _executors_map) {
        auto& executor = pair.second.executor;
        executor->cancel();
    }
}

void performance_test::System::wait_discovery() {
    // period at which PDP and EDP are checked
    std::chrono::milliseconds period = 30ms;
    // maximum discovery time, after which the experiment is shut down
    std::chrono::milliseconds max_discovery_time = 30s;

    wait_pdp_discovery(period, max_discovery_time);

    wait_edp_discovery(period, max_discovery_time);
}

void performance_test::System::wait_pdp_discovery(
    std::chrono::milliseconds period, std::chrono::milliseconds max_pdp_time) {
    // period at which PDP is checked
    rclcpp::WallRate rate(period);

    auto pdp_start_time = std::chrono::high_resolution_clock::now();

    auto get_intersection_size = [=](std::vector<std::string> A,
                                     std::vector<std::string> B) {
        // returns how many values are present in both A and B
        std::sort(A.begin(), A.end());
        std::sort(B.begin(), B.end());
        std::vector<std::string> v_intersection;
        std::set_intersection(A.begin(), A.end(), B.begin(), B.end(),
                              std::back_inserter(v_intersection));
        return v_intersection.size();
    };

    // create a vector with all the names of the nodes to be discovered
    std::vector<std::string> reference_names;
    for (const auto& n : _nodes) {
        std::string node_name = n->get_fully_qualified_name();
        reference_names.push_back(node_name);
    }

    // count the total number of nodes
    size_t num_nodes = _nodes.size();

    bool pdp_ok = false;
    while (!pdp_ok) {
        for (const auto& n : _nodes) {
            // we use the intersection to avoid counting nodes discovered from
            // other processes
            size_t discovered_participants =
                get_intersection_size(n->get_node_names(), reference_names);
            pdp_ok = (discovered_participants == num_nodes);
            if (!pdp_ok) break;
        }

        if (pdp_ok) break;

        // check if maximum discovery time exceeded
        auto t = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                            t - pdp_start_time - max_pdp_time)
                            .count();
        if (duration > 0) {
            assert(0 &&
                   "[discovery] PDP took more than maximum discovery time");
        }

        rate.sleep();
    }

    if (_events_logger != nullptr) {
        // Create an event for PDP completed
        EventsLogger::Event pdp_ev;
        pdp_ev.caller_name = "SYSTEM";
        pdp_ev.code = EventsLogger::EventCode::discovery;
        pdp_ev.description = "[discovery] PDP completed";
        _events_logger->write_event(pdp_ev);
    }
}

void performance_test::System::wait_edp_discovery(
    std::chrono::milliseconds period, std::chrono::milliseconds max_edp_time) {
    // period at which EDP is checked
    rclcpp::WallRate rate(period);

    auto edp_start_time = std::chrono::high_resolution_clock::now();

    // count the number of subscribers for each topic
    std::map<std::string, int> subs_per_topic;
    for (const auto& n : _nodes) {
        auto trackers = n->all_trackers();
        for (const auto& tracker : *trackers) {
            subs_per_topic[tracker.first] += 1;
        }
    }

    // TODO: the EDP should also take into account if subscriptions have been
    // matched with publishers This is needed in case of processes with only
    // subscriptions
    bool edp_ok = false;
    while (!edp_ok) {
        for (const auto& n : _nodes) {
            // if the node has no publishers, it will be skipped.
            // however, the boolean flag has to be set to true.
            if (n->_pubs.empty()) {
                edp_ok = true;
                continue;
            }
            for (const auto& pub_tracker : n->_pubs) {
                std::string topic_name = pub_tracker.first;
                int discovered_endpoints = n->count_subscribers(topic_name);
                // we check greater or equal to take into account for other
                // processes
                edp_ok = (discovered_endpoints >= subs_per_topic[topic_name]);

                if (!edp_ok) break;
            }

            if (!edp_ok) break;
        }

        if (edp_ok) break;

        // check if maximum discovery time exceeded
        auto t = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                            t - edp_start_time - max_edp_time)
                            .count();
        if (duration > 0) {
            assert(0 &&
                   "[discovery] EDP took more than maximum discovery time");
        }

        rate.sleep();
    }

    if (_events_logger != nullptr) {
        // Create an event for EDP completed
        EventsLogger::Event edp_ev;
        edp_ev.caller_name = "SYSTEM";
        edp_ev.code = EventsLogger::EventCode::discovery;
        edp_ev.description = "[discovery] EDP completed";
        _events_logger->write_event(edp_ev);
    }
}

void performance_test::System::save_latency_all_stats(
    const std::map<std::string, std::vector<double>>& system_latency_stats,
    std::string filename) const {
    if (filename.empty()) {
        std::cout << "[SystemLatencyLogger]: Error. Provided an empty filename."
                  << std::endl;
        std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
        return;
    }

    std::ofstream out_file;
    out_file.open(filename);

    if (!out_file.is_open()) {
        std::cout << "[SystemLatencyLogger]: Error. Could not open file "
                  << filename << std::endl;
        std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
        return;
    }

    this->log_latency_all_stats(system_latency_stats, out_file);
}

void performance_test::System::save_profiling_inter_node_stats(
    const std::string& filename) const {
    if (filename.empty()) {
        std::cout << "[SystemLatencyLogger]: Error. Provided an empty filename."
                  << std::endl;
        std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
        return;
    }

    std::ofstream out_file;
    out_file.open(filename);

    if (!out_file.is_open()) {
        std::cout << "[SystemLatencyLogger]: Error. Could not open file "
                  << filename << std::endl;
        std::cout << "[SystemLatencyLogger]: Not logging." << std::endl;
        return;
    }

    this->log_profiling_inter_node_stats(out_file);
}

void performance_test::System::save_profiling_first_to_last_node_stats(
    const std::map<std::string, std::vector<double>>& system_latency_stats,
    const std::string& filename) const {
    if (filename.empty()) {
        std::cout << "[SystemLatencyProfilingLogger]: Error. Provided an empty "
                     "filename."
                  << std::endl;
        std::cout << "[SystemLatencyProfilingLogger]: Not logging."
                  << std::endl;
        return;
    }

    std::ofstream out_file;
    out_file.open(filename);

    if (!out_file.is_open()) {
        std::cout
            << "[SystemLatencyProfilingLogger]: Error. Could not open file "
            << filename << std::endl;
        std::cout << "[SystemLatencyProfilingLogger]: Not logging."
                  << std::endl;
        return;
    }

    this->log_profiling_first_to_last_node_stats(system_latency_stats,
                                                 out_file);
}

void performance_test::System::print_latency_all_stats(
    const std::map<std::string, std::vector<double>>& system_latency_stats)
    const {
    this->log_latency_all_stats(system_latency_stats, std::cout);
}

void performance_test::System::log_latency_all_stats(
    const std::map<std::string, std::vector<double>>& system_latency_stats,
    std::ostream& stream) const {
    // log header
    stream << std::left << "node";
    stream << std::left << std::setw(_logging_parameters.wide_space)
           << std::setfill(' ') << _logging_parameters.separator << "topic";
    stream << std::left << std::setw(_logging_parameters.narrow_space)
           << std::setfill(' ') << _logging_parameters.separator << "size[b]";
    stream << std::left << std::setw(_logging_parameters.narrow_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "received[#]";
    stream << std::left << std::setw(_logging_parameters.narrow_space)
           << std::setfill(' ') << _logging_parameters.separator << "late[#]";
    stream << std::left << std::setw(_logging_parameters.wide_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "too_late[#]";
    stream << std::left << std::setw(_logging_parameters.narrow_space)
           << std::setfill(' ') << _logging_parameters.separator << "lost[#]";
    stream << std::left << std::setw(_logging_parameters.narrow_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "mean_inter_node[us]";
    stream << std::left << std::setw(_logging_parameters.narrow_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "sd_inter_node[us]";
    stream << std::left << std::setw(_logging_parameters.narrow_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "min_inter_node[us]";
    stream << std::left << std::setw(_logging_parameters.narrow_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "max_inter_node[us]";
    stream << std::left << std::setw(_logging_parameters.narrow_space)
           << std::setfill(' ') << _logging_parameters.separator << "freq[hz]";
    stream << std::left << std::setw(_logging_parameters.wide_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "duration[s]";
    stream << std::left << std::setw(_logging_parameters.wide_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "e2e_latency_mean[us]";
    stream << std::left << std::setw(_logging_parameters.wide_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "e2e_latency_std[us]";
    stream << std::endl;

    unsigned long int total_received = 0;
    unsigned long int total_lost = 0;
    unsigned long int total_late = 0;
    unsigned long int total_too_late = 0;

    // Print all
    for (const auto& n : _nodes) {
        auto trackers = n->all_trackers();
        for (const auto& tracker : *trackers) {
            total_received += tracker.second.received();
            total_lost += tracker.second.lost();
            total_late += tracker.second.late();
            total_too_late += tracker.second.too_late();

            stream << std::left << n->get_name();
            stream << std::left << std::setw(_logging_parameters.wide_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << tracker.first;
            stream << std::left << std::setw(_logging_parameters.narrow_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << tracker.second.size();
            stream << std::left << std::setw(_logging_parameters.wide_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << tracker.second.received();
            stream << std::left << std::setw(_logging_parameters.narrow_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << tracker.second.late();
            stream << std::left << std::setw(_logging_parameters.wide_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << tracker.second.too_late();
            stream << std::left << std::setw(_logging_parameters.narrow_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << tracker.second.lost();
            stream << std::left << std::setw(_logging_parameters.narrow_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << std::round(tracker.second
                                     .inter_node_latency_stats()["end-to-end"]
                                     .mean());
            stream << std::left << std::setw(_logging_parameters.narrow_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << std::round(tracker.second
                                     .inter_node_latency_stats()["end-to-end"]
                                     .stddev());
            stream << std::left << std::setw(_logging_parameters.narrow_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << std::round(tracker.second
                                     .inter_node_latency_stats()["end-to-end"]
                                     .min());
            stream << std::left << std::setw(_logging_parameters.narrow_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << std::round(tracker.second
                                     .inter_node_latency_stats()["end-to-end"]
                                     .max());
            stream << std::left << std::setw(_logging_parameters.narrow_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << tracker.second.frequency();
            stream << std::left << std::setw(_logging_parameters.wide_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << _experiment_duration_sec;
            stream << std::left << std::setw(_logging_parameters.wide_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << std::setprecision(4)
                   << system_latency_stats.at("end-to-end").at(0);
            stream << std::left << std::setw(_logging_parameters.wide_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << std::setprecision(4)
                   << system_latency_stats.at("end-to-end").at(1);
            stream << std::endl;
        }
    }
}

void performance_test::System::log_profiling_inter_node_stats(
    std::ostream& stream) const {
    // log header

    stream << std::left << "node";
    stream << std::left << std::setw(_logging_parameters.wide_space)
           << std::setfill(' ') << _logging_parameters.separator << "topic";
    log_latency_stats_header(stream);

    // Print all
    for (const auto& n : _nodes) {
        auto trackers = n->all_trackers();
        for (const auto& tracker : *trackers) {
            const auto& s = tracker.second.inter_node_latency_stats();
            stream << std::left << n->get_name();
            stream << std::left << std::setw(_logging_parameters.wide_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << tracker.first;

            for (const auto& profiling_label : _profiling_labels) {
                stream << std::left << std::setw(_logging_parameters.wide_space)
                        << std::setfill(' ') << _logging_parameters.separator
                        << s.at(profiling_label).mean();
                stream << std::left << std::setw(_logging_parameters.wide_space)
                        << std::setfill(' ') << _logging_parameters.separator
                        << s.at(profiling_label).stddev();
            }
            stream << std::left << std::setw(_logging_parameters.wide_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << s.at("end-to-end").mean();
            stream << std::left << std::setw(_logging_parameters.wide_space)
                   << std::setfill(' ') << _logging_parameters.separator
                   << s.at("end-to-end").stddev() << std::endl;
        }
    }
}

void performance_test::System::dump_absolute_profiling_timestamps(
    const std::string& file_basename) const {
    std::vector<std::string> absolute_timestamps_labels = {"header", "now"};
    absolute_timestamps_labels.insert(absolute_timestamps_labels.end(),
                                       _profiling_labels.begin(),
                                       _profiling_labels.end());
    auto all_trackers = get_trackers_of_node_system();

    for (std::string absolute_timestamps_label : absolute_timestamps_labels) {
        // create file. ...
        // Print all
        std::ofstream ofs(file_basename + "_" + absolute_timestamps_label + ".csv");

        // log header
        ofs << std::left << "publisher_tracker" << _logging_parameters.separator
            << _nodes.size() << std::endl;
        for (const auto& tracker : *all_trackers) {
            const auto& samples =
                tracker.second.profiling_timestamps_absolute().at(
                    absolute_timestamps_label);
            ofs << std::left << tracker.first << _logging_parameters.separator;
            dump_vector_data(samples, ofs);
        }
    }
}

void performance_test::System::dump_tracking_numbers(
    const std::string& file_basename) const {
    auto all_trackers = get_trackers_of_node_system();

    std::ofstream ofs(file_basename + ".csv");

    // log header
    ofs << std::left << "publisher_tracker" << _logging_parameters.separator
        << _nodes.size() << std::endl;
    for (const auto& tracker : *all_trackers) {
        ofs << std::left << tracker.first << _logging_parameters.separator;
        dump_vector_data(tracker.second.tracking_numbers(), ofs);
    }
}

void performance_test::System::log_latency_stats_header(
    std::ostream& stream) const {

    for (const auto& profiling_label : _profiling_labels) {
        stream << std::left << std::setw(_logging_parameters.wide_space)
            << std::setfill(' ') << _logging_parameters.separator
            << profiling_label << "_mean[us";
        stream << std::left << std::setw(_logging_parameters.wide_space)
            << std::setfill(' ') << _logging_parameters.separator
            << profiling_label << "_std[us]";
    }
    stream << std::left << std::setw(_logging_parameters.wide_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "e2e_latency_mean[us]";
    stream << std::left << std::setw(_logging_parameters.wide_space)
           << std::setfill(' ') << _logging_parameters.separator
           << "e2e_latency_std[us]";
    stream << std::endl;
}
void performance_test::System::log_profiling_first_to_last_node_stats(
    const std::map<std::string, std::vector<double>>& system_latency_stats,
    std::ostream& stream) const {
    // log header
    stream << std::left << "no_nodes";
    log_latency_stats_header(stream);

    // log actual content
    stream << std::left << _nodes.size();

    for (const auto& profiling_label: _profiling_labels) {
        stream << std::left << std::setw(_logging_parameters.wide_space)
            << std::setfill(' ') << _logging_parameters.separator
            << std::round(
                    system_latency_stats.at(profiling_label).at(0));
        stream << std::left << std::setw(_logging_parameters.wide_space)
            << std::setfill(' ') << _logging_parameters.separator
            << std::round(
                    system_latency_stats.at(profiling_label).at(1));
    }
    stream << std::left << std::setw(_logging_parameters.wide_space)
           << std::setfill(' ') << _logging_parameters.separator
           << std::round(system_latency_stats.at("end-to-end").at(0));
    stream << std::left << std::setw(_logging_parameters.wide_space)
           << std::setfill(' ') << _logging_parameters.separator
           << std::round(system_latency_stats.at("end-to-end").at(1))
           << std::endl;
}

// returns standard deviation of the oveall laency (element 0) and of the profiling timestamps
// it is assumed that the nodes are in a perfect pipeline.
// "all_trackers" is supposed to be all_trackers for the current data_processing pipeline!
std::map<std::string, std::vector<double>> performance_test::System::calc_first_to_last_node_latency_stats() const {
    std::map<std::string, std::vector<double>> stats;

    auto all_trackers = get_trackers_of_node_system();
    return calc_first_to_last_node_latency_stats(all_trackers);
}

std::map<std::string, std::vector<double>>
    performance_test::System::calc_first_to_last_node_latency_stats(
        const std::shared_ptr<Trackers> all_trackers) const {
    std::map<std::string, std::vector<double>> stats;

    std::map<std::string, std::vector<double>>
        first_to_last_node_added_latencies =
            sum_up_internode_latencies(all_trackers);

    for (const auto& iter : first_to_last_node_added_latencies) {
        stats.insert({iter.first, {mean(iter.second), stddev(iter.second)}});
    }
    return stats;
}

// sums up the internode latency stats to obtain a time difference between
// first and last node
std::map<std::string, std::vector<double>>
    performance_test::System::sum_up_internode_latencies(
        const std::shared_ptr<Trackers> trackers_node_system) const {
    int minimum_iterations = get_minimum_iterations(trackers_node_system);

    std::map<std::string, std::vector<double>>
        first_to_last_node_added_latencies =
            create_latency_stats_map_from_profiling_indices<std::vector<double>>();

    // to avoid confusions, we call the number of sent messages "iterations"
    for (int iteration = 0; iteration < minimum_iterations; iteration++) {
        // for all publisher subscriber combinations, we need to add the
        // internode latencies to know how large the latency and the profiling
        // is from the first to the last node
        std::map<std::string, double>
        first_to_last_node_latencies_curr_iteration =
            create_latency_stats_map_from_profiling_indices<double>();

        // this vector stores the samples we get considering the time elapsed
        // from publisher to subscriber, i.e. e2e. We use this because if there
        // is one message late in our iteration, we do not calculate any
        // statistics for the whole iteration
        std::vector<double> first_to_last_node_samples;

        // lets get all internode e2e latencies for this iteration...
        for (auto& tracker : *trackers_node_system) {
            first_to_last_node_samples.push_back(
                tracker.second.inter_node_latency_stats()["end-to-end"]
                    .samples()[iteration]);
        }
        // if there is no msg which is too late, let's do some calculations
        if (!is_corrupted_iteration(first_to_last_node_samples)) {
            // for easch tracker ....
            for (const auto& tracker : *trackers_node_system) {
                // .. go over each profiling step (= latency type) and add these
                // together to get the latencies from the first to the last node
                for (const auto& latency_type_stats :
                     tracker.second.inter_node_latency_stats()) {
                    first_to_last_node_latencies_curr_iteration
                        [latency_type_stats.first] +=
                        latency_type_stats.second.samples()[iteration];
                }
            }

            // add these values now as samples for the statistics later on
            for (const auto& iter :
                 first_to_last_node_latencies_curr_iteration) {
                first_to_last_node_added_latencies[iter.first].push_back(
                    iter.second);
            }
        } else {
            for (const auto& iter : first_to_last_node_latencies_curr_iteration)
                first_to_last_node_added_latencies[iter.first].push_back(
                    std::nan(""));
        }
    }
    return first_to_last_node_added_latencies;
}

std::map<std::string, std::vector<double>>
    performance_test::System::sum_up_internode_latencies() const {
    auto trackers_node_system = get_trackers_of_node_system();
    return sum_up_internode_latencies(trackers_node_system);
}

int performance_test::System::get_minimum_iterations(
    const std::shared_ptr<Trackers> all_trackers) const {
    int min_val = 9999999;
    for (auto& tracker : *all_trackers) {
        if (tracker.second.received() < min_val)
            min_val = tracker.second.received();
    }
    return min_val;
}

double performance_test::System::mean(
    const std::vector<double>& samples) const {
    double mean;
    if (samples.size() == 0)
        mean = std::nan("");
    else
        mean = std::accumulate(samples.begin(), samples.end(), 0.0) /
               samples.size();
    // if (mean == -1) mean = std::nan("");

    return mean;
}

double performance_test::System::stddev(
    const std::vector<double>& samples) const {
    double std;
    double variance = 0.0;
    if (samples.size() == 0)
        std = std::nan("");
    else if (samples.size() == 1)
        std = 0;
    else {
        std::vector<double> samples_squared;
        for (auto& sample : samples) {
            samples_squared.push_back(sample * sample);
        }

        double samples_mean = mean(samples);
        for (auto& sample : samples)
            variance += (sample - samples_mean) * (sample - samples_mean);

        variance = variance / (samples.size() - 1);
        std = std::sqrt(variance);
    }
    return std;
}

// For our use case, we assume that there will be always complete data processing pipelines.
// That means, our whole node system has only one end.
// This function collects for all nodes in our pipeline the
// trackers.
typedef std::vector<std::pair<std::string, performance_test::Tracker>> Trackers;
std::shared_ptr<Trackers>
    performance_test::System::get_trackers_of_node_system() const {
    Trackers all_trackers_node_system;
    for (auto& n : _nodes) {
        auto current_node_all_trackers = n->all_trackers();
        for (auto& tracker : *current_node_all_trackers) {
            all_trackers_node_system.push_back(tracker);
        }
    }
    return std::make_shared<Trackers>(all_trackers_node_system);
}

bool performance_test::System::is_corrupted_iteration(
    const std::vector<double>& samples) const {
    for (const double sample : samples) {
        if (std::isnan(sample)) return true;
    }
    return false;
}

void performance_test::System::dump_summed_up_internode_latencies(
    const std::map<std::string, std::vector<double>>& latencies,
    const std::string& filename) const {
    std::ofstream file(filename);
    file << "nodes," << _nodes.size() << std::endl;

    for (const auto& profiling_label : _profiling_labels) {
        file << profiling_label << ",";
        dump_vector_data<double>(latencies.at(profiling_label), file);
    }
    file << "e2e,";
    dump_vector_data<double>(latencies.at("end-to-end"), file);
}
