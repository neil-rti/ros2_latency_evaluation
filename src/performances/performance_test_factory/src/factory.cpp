/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <algorithm>
#include <map>

#include "performance_test/ros2/names_utilities.hpp"
#include "performance_test/ros2/node.hpp"

#include "performance_test_factory/factory.hpp"
#include "performance_test_factory/load_plugins.hpp"
#include "performance_test_factory/parse_data_processing_pipeline_section_from_json_node.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

std::shared_ptr<performance_test::Node>
    performance_test::TemplateFactory::create_node(
        std::string name, bool use_ipc, bool use_ros_params, bool verbose,
        std::string ros2_namespace, int executor_id) {
    std::shared_ptr<performance_test::Node> node;
    rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
    node_options.use_intra_process_comms(use_ipc);
    node_options.start_parameter_services(use_ros_params);
    node_options.start_parameter_event_publisher(use_ros_params);

    std::thread T([name, ros2_namespace, node_options, executor_id, &node]() {
        node = std::make_shared<performance_test::Node>(
            name, ros2_namespace, node_options, executor_id);
    });

    T.join();
    return node;
}

std::vector<std::shared_ptr<performance_test::Node>>
    performance_test::TemplateFactory::create_subscriber_nodes(
        int start_id, int end_id, int n_publishers, std::string msg_type,
        msg_pass_by_t msg_pass_by, Tracker::TrackingOptions tracking_options,
        rmw_qos_profile_t custom_qos_profile) {
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id++) {
        std::string node_name = id_to_node_name(node_id);
        auto node = this->create_node(node_name, _use_ipc, _use_ros_params,
                                      _verbose_mode, _ros2_namespace);

        // TODO: pass publisher list instead of n_publishers, to select the IDs
        // ( default is a list from 0 to n_pubs or directly from n_subs to
        // n_pubs)
        for (int k = 0; k < n_publishers; k++) {
            int topic_id = k + end_id;
            std::string sub_topic_name = id_to_topic_name(topic_id);
            std::string pub_topic_name = "";
            if (topic_id > 0) pub_topic_name = id_to_topic_name(topic_id);

            this->add_subscriber_from_strings(node, msg_type, sub_topic_name,
                                              pub_topic_name, tracking_options,
                                              msg_pass_by, custom_qos_profile);
        }

        nodes_vector.push_back(node);
    }

    return nodes_vector;
}

std::vector<std::shared_ptr<performance_test::Node>>
    performance_test::TemplateFactory::create_periodic_publisher_nodes(
        int start_id, int end_id, float frequency, std::string msg_type,
        msg_pass_by_t msg_pass_by, size_t msg_size,
        rmw_qos_profile_t custom_qos_profile) {
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id++) {
        std::string node_name = id_to_node_name(node_id);
        auto node = this->create_node(node_name, _use_ipc, _use_ros_params,
                                      _verbose_mode, _ros2_namespace);

        int topic_id = node_id;
        std::string topic_name = id_to_topic_name(topic_id);

        int period_us = (1000000 / frequency);
        std::chrono::microseconds period = std::chrono::microseconds(period_us);

        this->add_periodic_publisher_from_strings(
            node, msg_type, topic_name, msg_pass_by, custom_qos_profile, period,
            msg_size);

        nodes_vector.push_back(node);
    }

    return nodes_vector;
}

std::vector<std::shared_ptr<performance_test::Node>>
    performance_test::TemplateFactory::create_periodic_client_nodes(
        int start_id, int end_id, int n_services, float frequency,
        std::string srv_type, rmw_qos_profile_t custom_qos_profile) {
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id++) {
        std::string node_name = id_to_node_name(node_id);
        auto node = this->create_node(node_name, _use_ipc, _use_ros_params,
                                      _verbose_mode, _ros2_namespace);

        int period_us = (1000000 / frequency);
        std::chrono::microseconds period = std::chrono::microseconds(period_us);

        for (int k = 0; k < n_services; k++) {
            int service_id = k + end_id;
            std::string service_name = id_to_service_name(service_id);

            this->add_periodic_client_from_strings(node, srv_type, service_name,
                                                   custom_qos_profile, period);
        }

        nodes_vector.push_back(node);
    }

    return nodes_vector;
}

std::vector<std::shared_ptr<performance_test::Node>>
    performance_test::TemplateFactory::create_server_nodes(
        int start_id, int end_id, std::string srv_type,
        rmw_qos_profile_t custom_qos_profile) {
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vector;

    for (int node_id = start_id; node_id < end_id; node_id++) {
        std::string node_name = id_to_node_name(node_id);
        auto node = this->create_node(node_name, _use_ipc, _use_ros_params,
                                      _verbose_mode, _ros2_namespace);

        int service_id = node_id;
        std::string service_name = id_to_service_name(service_id);

        this->add_server_from_strings(node, srv_type, service_name,
                                      custom_qos_profile);

        nodes_vector.push_back(node);
    }

    return nodes_vector;
}

void performance_test::TemplateFactory::add_subscriber_from_strings(
    std::shared_ptr<performance_test::Node> n, std::string msg_type,
    std::string sub_topic_name, std::string pub_topic_name,
    Tracker::TrackingOptions tracking_options, msg_pass_by_t msg_pass_by,
    rmw_qos_profile_t custom_qos_profile) {
    auto library = performance_test::get_library(msg_type);

    typedef void (*function_impl_t)(std::shared_ptr<performance_test::Node>,
                                    std::string, std::string, std::string,
                                    Tracker::TrackingOptions, msg_pass_by_t,
                                    rmw_qos_profile_t);

    function_impl_t add_subscriber_impl =
        (function_impl_t)library->get_symbol("add_subscriber_impl");
    add_subscriber_impl(n, msg_type, sub_topic_name, pub_topic_name,
                        tracking_options, msg_pass_by, custom_qos_profile);
}

void performance_test::TemplateFactory::add_periodic_publisher_from_strings(
    std::shared_ptr<performance_test::Node> n, std::string msg_type,
    std::string topic_name, msg_pass_by_t msg_pass_by,
    rmw_qos_profile_t custom_qos_profile, std::chrono::microseconds period,
    size_t msg_size) {
    auto library = performance_test::get_library(msg_type);

    typedef void (*function_impl_t)(
        std::shared_ptr<performance_test::Node>, std::string, std::string,
        msg_pass_by_t, rmw_qos_profile_t, std::chrono::microseconds, size_t);

    function_impl_t add_publisher_impl =
        (function_impl_t)library->get_symbol("add_publisher_impl");
    add_publisher_impl(n, msg_type, topic_name, msg_pass_by, custom_qos_profile,
                       period, msg_size);
}

void performance_test::TemplateFactory::add_non_periodic_publisher_from_strings(
    std::shared_ptr<performance_test::Node> n, std::string msg_type,
    std::string sub_topic_name, std::string pub_topic_name,
    msg_pass_by_t msg_pass_by, rmw_qos_profile_t custom_qos_profile,
    size_t msg_size) {
    auto library = performance_test::get_library(msg_type);

    typedef void (*function_impl_t)(std::shared_ptr<performance_test::Node>,
                                    std::string, std::string, std::string,
                                    msg_pass_by_t, rmw_qos_profile_t, size_t);

    function_impl_t add_non_periodic_publisher_impl =
        (function_impl_t)library->get_symbol("add_non_periodic_publisher_impl");
    add_non_periodic_publisher_impl(n, msg_type, sub_topic_name, pub_topic_name,
                                    msg_pass_by, custom_qos_profile, msg_size);
}

void performance_test::TemplateFactory::add_server_from_strings(
    std::shared_ptr<performance_test::Node> n, std::string srv_type,
    std::string service_name, rmw_qos_profile_t custom_qos_profile) {
    auto library = performance_test::get_library(srv_type);

    typedef void (*function_impl_t)(std::shared_ptr<performance_test::Node>,
                                    std::string, std::string,
                                    rmw_qos_profile_t);

    function_impl_t add_server_impl =
        (function_impl_t)library->get_symbol("add_server_impl");
    add_server_impl(n, srv_type, service_name, custom_qos_profile);
}

void performance_test::TemplateFactory::add_periodic_client_from_strings(
    std::shared_ptr<performance_test::Node> n, std::string srv_type,
    std::string service_name, rmw_qos_profile_t custom_qos_profile,
    std::chrono::microseconds period) {
    auto library = performance_test::get_library(srv_type);

    typedef void (*function_impl_t)(std::shared_ptr<performance_test::Node>,
                                    std::string, std::string, rmw_qos_profile_t,
                                    std::chrono::microseconds period);

    function_impl_t add_client_impl =
        (function_impl_t)library->get_symbol("add_client_impl");
    add_client_impl(n, srv_type, service_name, custom_qos_profile, period);
}

std::vector<std::shared_ptr<performance_test::Node>>
    performance_test::TemplateFactory::parse_topology_from_json(
        std::string json_path, Tracker::TrackingOptions tracking_options) {
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vec;
    json topology_j = read_json_file(json_path);
    nodes_vec = parse_json_to_node_system(topology_j, tracking_options);
    return nodes_vec;
}

nlohmann::json performance_test::TemplateFactory::read_json_file(
    const std::string& json_path) {
    json ret_j;
    std::ifstream ifs(json_path);
    // Check if file exists
    if (!ifs.good()) {
        std::cout << "ERROR. Can't find file: " << json_path << std::endl;
        ret_j["ERROR"] = 1;
    }
    ret_j = json::parse(ifs);
    return ret_j;
}

std::vector<std::shared_ptr<performance_test::Node>>
    performance_test::TemplateFactory::parse_json_to_node_system(
        const nlohmann::json& topology_j,
        Tracker::TrackingOptions tracking_options) {
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vec;

    if (topology_j.contains("ERROR")) return nodes_vec;

    if (topology_j.find("nodes") == topology_j.end()) {
        std::cout << "ERROR. The provided json does not contain a nodes field"
                  << std::endl;
        return nodes_vec;
    }
    auto nodes_json = topology_j["nodes"];

    for (auto node_json : nodes_json) {
        if (node_json.find("number") != node_json.end()) {
            int number_of_nodes = node_json["number"];

            for (int node_number = 1; node_number <= number_of_nodes;
                 ++node_number) {
                std::string node_name_suffix =
                    '_' + std::to_string(node_number);
                auto node = create_node_from_json(node_json, node_name_suffix);
                create_node_entities_from_json(node, node_json,
                                               tracking_options);
                nodes_vec.push_back(node);
            }
        }
        else if (node_json.contains("data_processing_pipeline_nodes"))
        {
            ParseDataProcessingPipelineSectionFromJsonNode p(node_json);
            json data_processing_pipeline_json = p.run();

            for (auto node_json : data_processing_pipeline_json)
            {
                auto node = create_node_from_json(node_json);
                create_node_entities_from_json(node, node_json,
                                               tracking_options);
                nodes_vec.push_back(node);
            }
        } else {
            auto node = create_node_from_json(node_json);
            create_node_entities_from_json(node, node_json, tracking_options);
            nodes_vec.push_back(node);
        }
    }
    return nodes_vec;
}

std::shared_ptr<performance_test::Node>
    performance_test::TemplateFactory::create_node_from_json(
        const json node_json, std::string suffix) {
    auto node_name = std::string(node_json["node_name"]) + suffix;

    int executor_id = 0;
    if (node_json.find("executor_id") != node_json.end()) {
        executor_id = node_json["executor_id"];
    }

    auto node = this->create_node(node_name, _use_ipc, _use_ros_params,
                                  _verbose_mode, _ros2_namespace, executor_id);

    return node;
}

void performance_test::TemplateFactory::create_node_entities_from_json(
    std::shared_ptr<performance_test::Node> node, const json node_json,
    Tracker::TrackingOptions tracking_options) {
    if (node_json.find("initial_pub") != node_json.end()) {
        // if there is at least 1 publisher, add each of them
        for (auto p_json : node_json["initial_pub"]) {
            this->add_periodic_publisher_from_json(node, p_json);
        }
    }

    if (node_json.find("publishers") != node_json.end())
        this->add_non_periodic_publisher_from_json(node, node_json);

    if (node_json.find("subscribers") != node_json.end())
        this->add_subscriber_from_json(node, node_json, tracking_options);

    if (node_json.find("clients") != node_json.end()) {
        // if there is at least 1 client, add each of them
        for (auto c_json : node_json["clients"]) {
            this->add_periodic_client_from_json(node, c_json);
        }
    }

    if (node_json.find("servers") != node_json.end()) {
        // if there is at least 1 server, add each of them
        for (auto s_json : node_json["servers"]) {
            this->add_server_from_json(node, s_json);
        }
    }
}

void performance_test::TemplateFactory::add_periodic_publisher_from_json(
    std::shared_ptr<performance_test::Node> node, const json pub_json) {
    std::string topic_name = pub_json["topic_name"];
    std::string msg_type = pub_json["msg_type"];

    float period_ms = 0;

    if (pub_json.find("freq_hz") != pub_json.end()) {
        float frequency = pub_json["freq_hz"];
        period_ms = 1000 / frequency;
    } else if (pub_json.find("period_ms") != pub_json.end()) {
        period_ms = pub_json["period_ms"];
    } else {
        std::cout
            << "Error! Publishers must set period_ms or freq_hz in json file"
            << std::endl;
    }

    auto period = std::chrono::microseconds(static_cast<int>(period_ms * 1000));

    size_t msg_size = 0;
    if (pub_json.find("msg_size") != pub_json.end()) {
        msg_size = pub_json["msg_size"];
    }

    rmw_qos_profile_t custom_qos_profile = get_qos_from_json(pub_json);

    msg_pass_by_t msg_pass_by =
        get_msg_pass_by_from_json(pub_json, PASS_BY_UNIQUE_PTR);

    this->add_periodic_publisher_from_strings(node, msg_type, topic_name,
                                              msg_pass_by, custom_qos_profile,
                                              period, msg_size);
}

void performance_test::TemplateFactory::add_non_periodic_publisher_from_json(
    std::shared_ptr<performance_test::Node> node, const json node_json) {
    std::string pub_topic_name = node_json["publishers"][0]["topic_name"];
    std::string sub_topic_name = node_json["subscribers"][0]["topic_name"];
    std::string msg_type = node_json["publishers"][0]["msg_type"];

    size_t msg_size = 0;
    if (node_json["publishers"][0].find("msg_size") !=
        node_json["publishers"][0].end()) {
        msg_size = node_json["publishers"][0]["msg_size"];
    }

    nlohmann::json pub_json = node_json["publishers"][0];
    rmw_qos_profile_t custom_qos_profile = get_qos_from_json(pub_json);

    msg_pass_by_t msg_pass_by =
        get_msg_pass_by_from_json(pub_json, PASS_BY_UNIQUE_PTR);

    this->add_non_periodic_publisher_from_strings(
        node, msg_type, sub_topic_name, pub_topic_name, msg_pass_by,
        custom_qos_profile, msg_size);
}

void performance_test::TemplateFactory::add_subscriber_from_json(
    std::shared_ptr<performance_test::Node> node, const json node_json,
    Tracker::TrackingOptions t_options) {
    std::string sub_topic_name = node_json["subscribers"][0]["topic_name"];
    std::string pub_topic_name = "";
    if (!node_json.contains(
            "publishers"))  // we reached the last node ->
                            // pub_topic_name == sub_topic_name!!
        pub_topic_name = sub_topic_name;
    else
        pub_topic_name = node_json["publishers"][0]["topic_name"];

    std::string msg_type = node_json["subscribers"][0]["msg_type"];

    nlohmann::json sub_json = node_json["subscribers"][0];
    rmw_qos_profile_t custom_qos_profile = get_qos_from_json(sub_json);

    msg_pass_by_t msg_pass_by =
        get_msg_pass_by_from_json(sub_json, PASS_BY_UNIQUE_PTR);

    this->add_subscriber_from_strings(node, msg_type, sub_topic_name,
                                      pub_topic_name, t_options, msg_pass_by,
                                      custom_qos_profile);
}

void performance_test::TemplateFactory::add_periodic_client_from_json(
    std::shared_ptr<performance_test::Node> node, const json client_json) {
    std::string service_name = client_json["service_name"];
    std::string srv_type = client_json["srv_type"];

    float period_ms;

    if (client_json.find("freq_hz") != client_json.end()) {
        float frequency = client_json["freq_hz"];
        period_ms = 1000 / frequency;
    } else if (client_json.find("period_ms") != client_json.end()) {
        period_ms = client_json["period_ms"];
    } else {
        std::cout << "Error! Clients must set period_ms or freq_hz in json file"
                  << std::endl;
    }

    auto period = std::chrono::microseconds(static_cast<int>(period_ms * 1000));

    rmw_qos_profile_t custom_qos_profile = get_qos_from_json(client_json);

    this->add_periodic_client_from_strings(node, srv_type, service_name,
                                           custom_qos_profile, period);
}

void performance_test::TemplateFactory::add_server_from_json(
    std::shared_ptr<performance_test::Node> node, const json server_json) {
    std::string service_name = server_json["service_name"];
    std::string srv_type = server_json["srv_type"];
    rmw_qos_profile_t custom_qos_profile = get_qos_from_json(server_json);

    this->add_server_from_strings(node, srv_type, service_name,
                                  custom_qos_profile);
}

rmw_qos_profile_t performance_test::TemplateFactory::get_qos_from_json(
    const json entity_json) {
    // Create custom QoS profile with default values
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

    // Crete map for each QoS
    std::map<std::string, rmw_qos_history_policy_t> history_qos_map{
        {"system_default", RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT},
        {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
        {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL},
        {"unknown", RMW_QOS_POLICY_HISTORY_UNKNOWN}};

    std::map<std::string, rmw_qos_reliability_policy_t> reliability_qos_map{
        {"system_default", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT},
        {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
        {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT},
        {"unknown", RMW_QOS_POLICY_RELIABILITY_UNKNOWN}};

    std::map<std::string, rmw_qos_durability_policy_t> durability_qos_map{
        {"system_default", RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT},
        {"transient_local", RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
        {"volatile", RMW_QOS_POLICY_DURABILITY_VOLATILE},
        {"unknown", RMW_QOS_POLICY_DURABILITY_UNKNOWN},
    };

    std::map<std::string, rmw_qos_liveliness_policy_t> liveliness_qos_map{
        {"system_default", RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT},
        {"automatic", RMW_QOS_POLICY_LIVELINESS_AUTOMATIC},
        {"manual_by_node", RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE},
        {"manual_by_topic", RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC},
        {"unknown", RMW_QOS_POLICY_LIVELINESS_UNKNOWN}};

    std::map<std::string, bool> namespace_conventions_qos_map{{"false", false},
                                                              {"true", true}};

    std::map<std::string, struct rmw_time_t> deadline_qos_map{
        {"default", RMW_QOS_DEADLINE_DEFAULT}};

    std::map<std::string, struct rmw_time_t> lifespan_qos_map{
        {"default", RMW_QOS_LIFESPAN_DEFAULT}};

    std::map<std::string, struct rmw_time_t> liveliness_lease_duration_qos_map{
        {"default", RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT}};

    // Look in the entity json file for QoS settings
    if (entity_json.find("qos_history") != entity_json.end()) {
        custom_qos_profile.history =
            history_qos_map[entity_json["qos_history"]];
    }

    if (entity_json.find("qos_depth") != entity_json.end()) {
        custom_qos_profile.depth = (size_t)entity_json["qos_depth"];
    }

    if (entity_json.find("qos_reliability") != entity_json.end()) {
        custom_qos_profile.reliability =
            reliability_qos_map[entity_json["qos_reliability"]];
    }

    if (entity_json.find("qos_durability") != entity_json.end()) {
        custom_qos_profile.durability =
            durability_qos_map[entity_json["qos_durability"]];
    }

    if (entity_json.find("qos_liveliness") != entity_json.end()) {
        custom_qos_profile.liveliness =
            liveliness_qos_map[entity_json["qos_liveliness"]];
    }

    if (entity_json.find("qos_avoid_ros_namespace_conventions") !=
        entity_json.end()) {
        custom_qos_profile.avoid_ros_namespace_conventions =
            namespace_conventions_qos_map
                [entity_json["qos_avoid_ros_namespace_conventions"]];
    }

    if (entity_json.find("qos_deadline") != entity_json.end()) {
        custom_qos_profile.deadline =
            deadline_qos_map[entity_json["qos_deadline"]];
    }

    if (entity_json.find("qos_lifespan") != entity_json.end()) {
        custom_qos_profile.lifespan =
            lifespan_qos_map[entity_json["qos_lifespan"]];
    }

    if (entity_json.find("qos_liveliness_lease_duration") !=
        entity_json.end()) {
        custom_qos_profile.liveliness_lease_duration =
            liveliness_lease_duration_qos_map
                [entity_json["qos_liveliness_lease_duration"]];
    }

    return custom_qos_profile;
}

msg_pass_by_t performance_test::TemplateFactory::get_msg_pass_by_from_json(
    const json entity_json, msg_pass_by_t default_value) {
    msg_pass_by_t msg_pass_by = default_value;

    std::map<std::string, msg_pass_by_t> map_msg_pass_by{
        {"unique_ptr", PASS_BY_UNIQUE_PTR}, {"shared_ptr", PASS_BY_UNIQUE_PTR}};

    if (entity_json.find("msg_pass_by") != entity_json.end()) {
        msg_pass_by = map_msg_pass_by[entity_json["msg_pass_by"]];
    }

    return msg_pass_by;
}

void performance_test::TemplateFactory::change_data_processing_pipeline(json& node_system_j, const unsigned int number) {
    for (auto &node_j : node_system_j["nodes"])
    {
        if (node_j.contains("data_processing_pipeline_nodes"))
            node_j["data_processing_pipeline_nodes"] = number;
    }
}

void performance_test::TemplateFactory::change_payload_size(
    const std::string& payload_size, json& node_system_j) {
    if (payload_size != "") {
        for (auto& node_j : node_system_j["nodes"]) {
            if (node_j.contains("publishers")) {
                for (auto& publisher_j : node_j["publishers"]) {
                    publisher_j["msg_type"] = payload_size;
                }
            }
            if (node_j.contains("initial_pub")) {
                for (auto& publisher_j : node_j["initial_pub"]) {
                    publisher_j["msg_type"] = payload_size;
                }
            }

            if (node_j.contains("subscribers_start_node"))
              node_j["subscribers_start_node"]["msg_type"] = payload_size;
            
            if (node_j.contains("publisher_data_processing_pipeline_node"))
              node_j["publisher_data_processing_pipeline_node"]["msg_type"] = payload_size;

            if (node_j.contains("publisher_end_node"))
              node_j["publishers_end_node"]["msg_type"] = payload_size;

            if (node_j.contains("subscribers")) {
                for (auto& subscriber_j : node_j["subscribers"]) {
                    subscriber_j["msg_type"] = payload_size;
                }
            }
        }
    }
}

void performance_test::TemplateFactory::change_qos_policy_value(
    const std::string& qos_policy, const std::string& qos_value,
    json& node_system_j) {
    if (qos_value != "") {
        for (auto& node_j : node_system_j["nodes"]) {
            if (node_j.contains("publishers")) {
                for (auto& publisher_j : node_j["publishers"]) {
                    publisher_j[qos_policy] = qos_value;
                }
            }
            if (node_j.contains("initial_pub")) {
                for (auto& publisher_j : node_j["initial_pub"]) {
                    publisher_j["qos_policy"] = qos_value;
                }
            }
            if (node_j.contains("subscribers_start_node"))
                node_j["subscribers_start_node"][qos_policy] = qos_value;

            if (node_j.contains("publisher_data_processing_pipeline_node"))
              node_j["publisher_data_processing_pipeline_node"][qos_policy] = qos_value;

            if (node_j.contains("publishers_end_node"))
                node_j["publishers_end_node"][qos_policy] = qos_value;

            if (node_j.contains("subscribers")) {
                for (auto& subscriber_j : node_j["subscribers"])
                    subscriber_j[qos_policy] = qos_value;
            }
        }
    }
}

void performance_test::TemplateFactory::change_publish_period_ms(
    const int publish_period_ms, json& node_system_j) {
    if (publish_period_ms > 0) {
        for (auto& node_j : node_system_j["nodes"]) {
            if (node_j.contains("publishers")) {
                for (auto& publisher_j : node_j["publishers"]) {
                    publisher_j["period_ms"] = publish_period_ms;
                }
            }
            if (node_j.contains("initial_pub")) {
                for (auto& publisher_j : node_j["initial_pub"]) {
                    publisher_j["period_ms"] = publish_period_ms;
                }
            }
            
            if (node_j.contains("publisher_data_processing_pipeline_node"))
              node_j["publisher_data_processing_pipeline_node"]["period_ms"] = publish_period_ms;

            if (node_j.contains("publishers_end_node"))
                node_j["publishers_end_node"]["period_ms"] = publish_period_ms;
        }
    }
}

std::vector<std::string>
    performance_test::TemplateFactory::read_payloads_from_file(
        const std::string& file_path) const {
    std::vector<std::string> payloads;

    if (file_path != "") {
        std::ifstream ifs(file_path);
        // Check if file exists
        if (!ifs.good()) {
            std::cout << "ERROR. Can't find file: " << file_path << std::endl;
            exit(EXIT_FAILURE);
        }

        std::string current_line;
        while (std::getline(ifs, current_line)) {
            payloads.push_back(current_line);
        }
    } else
        payloads.push_back("");

    return payloads;
}