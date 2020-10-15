/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#pragma once

#include <iostream>
#include <map>

#include "rclcpp/time.hpp"

#include "performance_test/ros2/events_logger.hpp"
#include "performance_test/ros2/stat.hpp"
#include "performance_test/ros2/utils.hpp"

#include <ros2profiling/profiling.h>
#include "performance_test_msgs/msg/performance_header.hpp"

namespace performance_test {

class Tracker {
   public:
    struct TrackingOptions {
        TrackingOptions(bool enable = true) : is_enabled(enable){};

        bool is_enabled = true;
        int late_percentage = 20;
        int late_absolute_us = 5000;
        int too_late_percentage = 100;
        int too_late_absolute_us = 50000;
        bool enable_profiling = false;
    };

    typedef uint32_t TrackingNumber;

    Tracker() = delete;

    Tracker(std::string node_name, std::string topic_srv_name,
            TrackingOptions tracking_options)
        : _node_name(node_name),
          _topic_srv_name(topic_srv_name),
          _tracking_options(tracking_options) {
        _inter_node_latency_stats =
            create_latency_stats_map_from_profiling_indices<Stat<uint64_t>>();
        _profiling_indices = get_profiling_indices();
    }

    template<typename MsgType>
    void scan(const performance_test_msgs::msg::PerformanceHeader& header,
              const rclcpp::Time& now, const MsgType& msg,
              std::shared_ptr<EventsLogger> elog) {
        // If this is first message received store some info about it
        if (_inter_node_latency_stats["end-to-end"].n() == 0) {
            _size = header.size;
            _frequency = header.frequency;
            int period_us = 1000000 / _frequency;

            _latency_late_threshold_us =
                std::min(_tracking_options.late_absolute_us,
                         _tracking_options.late_percentage * period_us / 100);
            _latency_too_late_threshold_us = std::min(
                _tracking_options.too_late_absolute_us,
                _tracking_options.too_late_percentage * period_us / 100);
        }
        // Compute latency
        rclcpp::Time stamp(header.stamp.sec, header.stamp.nanosec,
                           RCL_SYSTEM_TIME);
        auto lat = std::chrono::nanoseconds((now - stamp).nanoseconds());
        uint64_t lat_us = static_cast<uint64_t>(lat.count()) / 1000;

        // store the last latency to be read from node
        _last_latency = lat_us;
        _tracking_number_memory.push_back(header.tracking_number);
        bool too_late = false;
        bool late = false;
        if (_tracking_options.is_enabled) {
            // Check if we received the correct message. The assumption here is
            // that the messages arrive in chronological order

            if (header.tracking_number == _tracking_number_count)
                _tracking_number_count++;
            else {
                // We missed some mesages...
                long unsigned int n_lost =
                    header.tracking_number - _tracking_number_count;
                _lost_messages += n_lost;
                _tracking_number_count = header.tracking_number + 1;

                // Log the event
                if (elog != nullptr) {
                    EventsLogger::Event ev;
                    std::stringstream description;
                    ev.caller_name = _topic_srv_name + "->" + _node_name;
                    ev.code = EventsLogger::EventCode::lost_messages;

                    description << "msgs " << header.tracking_number - 1
                                << " to " << header.tracking_number - 1 + n_lost
                                << " lost.";
                    ev.description = description.str();
                    elog->write_event(ev);
                }
            }
            too_late = _msg_too_late(lat_us);
            late = _msg_late(lat_us);

            if (late) {
                if (elog != nullptr) {
                    // Create a description for the event
                    std::stringstream description;
                    description << "msg " << header.tracking_number << " late. "
                                << lat_us << "us > "
                                << _latency_late_threshold_us << "us";

                    EventsLogger::Event ev;
                    ev.caller_name = _topic_srv_name + "->" + _node_name;
                    ev.code = EventsLogger::EventCode::late_message;
                    ev.description = description.str();

                    elog->write_event(ev);
                }
                _late_messages++;
            }

            if (too_late) {
                if (elog != nullptr) {
                    // Create a descrption for the event
                    std::stringstream description;
                    description << "msg " << header.tracking_number
                                << " too late. " << lat_us << "us > "
                                << _latency_too_late_threshold_us << "us";

                    EventsLogger::Event ev;
                    ev.caller_name = _topic_srv_name + "->" + _node_name;
                    ev.code = EventsLogger::EventCode::too_late_message;
                    ev.description = description.str();

                    elog->write_event(ev);
                }
                _too_late_messages++;
            }
        }
        _add_sample_to_internode_latency_stats(msg, lat_us, header, now);

        _received_messages++;
    }

    unsigned long int lost() const { return _lost_messages; }

    unsigned long int late() const { return _late_messages; }

    unsigned long int too_late() const { return _too_late_messages; }

    unsigned long int received() const { return _received_messages; }

    size_t size() const { return _size; }

    float frequency() const { return _frequency; }

    std::map<std::string, Stat<uint64_t>> inter_node_latency_stats() const {
        return _inter_node_latency_stats;
    }
    std::map<std::string, std::vector<uint64_t>> profiling_timestamps_absolute()
        const {
        return _profiling_timestamps_absolute;
    }
    std::vector<int> tracking_numbers() const {
        return _tracking_number_memory;
    }
    void set_frequency(const float f) { _frequency = f; }

    uint64_t last() const { return _last_latency; }

   private:
    std::string _node_name;
    std::string _topic_srv_name;

    uint64_t _last_latency = 0;
    unsigned long int _lost_messages = 0;
    unsigned long int _received_messages = 0;
    unsigned long int _late_messages = 0;
    unsigned long int _too_late_messages = 0;

    unsigned int _latency_late_threshold_us = 0;
    unsigned int _latency_too_late_threshold_us = 0;

    size_t _size = 0;
    float _frequency = 0;
    std::vector<int> _tracking_number_memory = {};
    std::map<std::string, Stat<uint64_t>> _inter_node_latency_stats;

    std::map<std::string, std::vector<uint64_t>>
        _profiling_timestamps_absolute = {{"header", {}},
                                          {"now", {}},
                                          {"rclcpp_interprocess_publish", {}},
                                          {"rcl_publish", {}},
                                          {"rmw_publish", {}},
                                          {"dds_write", {}},
                                          {"sub_dds_on_data", {}},
                                          {"sub_rclcpp_take_enter", {}},
                                          {"sub_rcl_take_enter", {}},
                                          {"sub_rmw_take_enter", {}},
                                          {"sub_dds_take_enter", {}},
                                          {"sub_dds_take_leave", {}},
                                          {"sub_rmw_take_leave", {}},
                                          {"sub_rcl_take_leave", {}},
                                          {"sub_rclcpp_take_leave", {}},
                                          {"rclcpp_handle", {}}};

    TrackingNumber _tracking_number_count = 0;
    TrackingOptions _tracking_options;

    bool _msg_late(const uint64_t& lat_us) {
        return (lat_us > _latency_late_threshold_us && !_msg_too_late(lat_us));
    }
    bool _msg_too_late(const uint64_t& lat_us) {
        return lat_us > _latency_too_late_threshold_us;
    }

    template<typename MsgType>
    void _add_sample_to_internode_latency_stats(
        const MsgType& msg, const uint64_t& lat_us,
        const performance_test_msgs::msg::PerformanceHeader& header,
        const rclcpp::Time& now) {
        rclcpp::Time stamp(header.stamp.sec, header.stamp.nanosec,
                           RCL_SYSTEM_TIME);

        _inter_node_latency_stats["end-to-end"].add_sample(lat_us);
        _profiling_timestamps_absolute["header"].push_back(stamp.nanoseconds());
        _profiling_timestamps_absolute["now"].push_back(now.nanoseconds());

        for (auto profiling_idx : _profiling_indices) {
            uint64_t delay_us = std::nanl("");
            uint64_t profiling_timestamp = std::nanl("");
            if (!std::isnan(lat_us) &&
                _tracking_options
                    .enable_profiling) {  // not too late or lost messages
                profiling_timestamp = get_profile(msg.get(), profiling_idx.second);
                if (profiling_idx.second == 0)
                    delay_us = static_cast<uint64_t>(
                        (profiling_timestamp - stamp.nanoseconds()) / 1e3);
                else
                    delay_us = static_cast<uint64_t>(
                        (profiling_timestamp -
                         get_profile(msg.get(), profiling_idx.second - 1)) /
                        1e3);
            }

            _inter_node_latency_stats[profiling_idx.first].add_sample(delay_us);
            _profiling_timestamps_absolute[profiling_idx.first].push_back(
                profiling_timestamp);
        }
    }
    std::map<std::string, int> _profiling_indices;
};
}  // namespace performance_test
