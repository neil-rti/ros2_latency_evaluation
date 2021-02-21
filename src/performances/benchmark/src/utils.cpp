#include <limits.h>
#include <unistd.h>
#include <ctime>
#include <string>
#include "nlohmann/json.hpp"

#include "cli/options.hpp"

using json = nlohmann::json;

std::string get_basename(const std::string& filename,
                         const std::string& file_extension) {
    const std::string file_extension_with_dot = "." + file_extension;
    const size_t last_slash = filename.find_last_of("/");
    const size_t file_suffix_start = filename.rfind(file_extension_with_dot);
    const size_t length_of_basename = file_suffix_start - (last_slash + 1);

    std::string basename = filename.substr(last_slash + 1, length_of_basename);
    return basename;
}

std::string create_results_dir_name(const benchmark::Options& options) {
    std::string dir_name;

    // get current time stamp
    time_t now;
    struct tm* timeinfo;
    char timestamp[100];

    time(&now);
    timeinfo = localtime(&now);

    strftime(timestamp, 100, "%Y-%m-%d_%H-%M", timeinfo);

    dir_name = std::string(timestamp);

    if (options.payload_file != "")
        dir_name += "_" + get_basename(options.payload_file, "txt");

    if (options.publish_frequency != 0)
        dir_name += "_PublishFrequency" +
                    std::to_string(options.publish_frequency) + "Hz";

    if (options.data_processing_pipeline_range_start != options.data_processing_pipeline_range_end)
      dir_name += "_DPPRange" 
                    + std::to_string(options.data_processing_pipeline_range_start) + "-" 
                    + std::to_string(options.data_processing_pipeline_range_end) + "-"
                    + std::to_string(options.data_processing_pipeline_range_step_size) + "_";

    if (options.msg_size != "") dir_name += "_MsgSize" + options.msg_size;

    dir_name += "_" + options.middle_ware;

    if (options.ipc)
        dir_name += "_IpcOn";
    else
        dir_name += "_IpcOff";

    if (options.tracking_options.enable_profiling)
        dir_name += "_ProfilingOn";
    else
        dir_name += "_ProfilingOff";

    dir_name += "_" +
                std::to_string(options.tracking_options.too_late_absolute_us) +
                "us";

    if (options.qos_reliability != "")
        dir_name += "_" + options.qos_reliability;
    if (options.experiment_name != "" &&
        options.experiment_name != "experiment")
        dir_name += "_" + options.experiment_name;
    return dir_name;
}

void print_user_info(const benchmark::Options& options, auto& json_list) {
    std::cout << "Topology file(s): " << std::endl;
    for(const auto& json : options.topology_json_list) std::cout << json << std::endl;

    std::cout << "Intra-process-communication: " << (options.ipc ? "on" : "off") << std::endl;
    std::cout << "Parameter services: " << (options.ros_params ? "on" : "off") << std::endl;
    std::cout << "Run test for: " << options.duration_sec << " seconds" << std::endl;
    std::cout << "Sampling resources every " << options.resources_sampling_per_ms << "ms" << std::endl;
    std::cout << "Logging events statistics: " << (options.tracking_options.is_enabled ? "on" : "off") << std::endl;
    std::cout << "data_processing_pipeline range between " << options.data_processing_pipeline_range_start << " and " << options.data_processing_pipeline_range_end << std::endl;
    std::cout << "Step size of data_processing_pipeline range is " << options.data_processing_pipeline_range_step_size
              << std::endl;
    std::cout << "The payload sizes will be changed to: " << options.msg_size
              << std::endl;
    std::cout << "payload variations will be from this file: "
              << options.payload_file << std::endl;
    std::cout << "profiling is activated: "
              << options.tracking_options.enable_profiling << std::endl;
    std::cout << "Chosen DDS Middleware: " << options.middle_ware << std::endl;
    std::cout << "Publish frequency is " << options.publish_frequency << "Hz"
              << std::endl;
    std::cout << "QoS Reliability: " << options.qos_reliability << std::endl;
    std::cout << "Start test" << std::endl;
}

std::string get_ws_folder_from_benchmark_folder(const std::string& cwd) {
    std::string workspace_folder = "";

    // go three directories up
    const size_t beginning_install_folder = cwd.find("/install/benchmark/lib");
    if (beginning_install_folder == std::string::npos)
        std::cout << "Couldn't find install folder, i.e. workspace"
                  << std::endl;
    else {
        workspace_folder = cwd.substr(0, beginning_install_folder);
    }

    return workspace_folder;
}

void set_env_dds_middleware(const std::string& ws_root,
                            const std::string& middle_ware) {
    std::string rmw_implementation_env = "rmw_" + middle_ware + "_cpp";
    std::string cyclone_dds_uri_env = ws_root + "/config/qos_cyclonedds.xml";
    std::string fastrtps_default_profiles_file_env =
        ws_root + "/config/qos_fastrtps.xml";
    std::string rmw_fastrtps_use_qos_from_xml_env = "1";
    std::string ndds_qos_profiles_env =
        "[" + ws_root + "/config/qos_connextdds.xml]";

    if(middle_ware == "connextdds") {
        rmw_implementation_env = "rmw_" + middle_ware;
    }
    setenv("RMW_IMPLEMENTATION", rmw_implementation_env.c_str(), 1);
    setenv("CYCLONEDDS_URI", cyclone_dds_uri_env.c_str(), 1);
    setenv("FASTRTPS_DEFAULT_PROFILES_FILE",
           fastrtps_default_profiles_file_env.c_str(), 1);
    setenv("RMW_FASTRTPS_USE_QOS_FROM_XML",
           rmw_fastrtps_use_qos_from_xml_env.c_str(), 1);
    setenv("NDDS_QOS_PROFILES", ndds_qos_profiles_env.c_str(), 1);
}

void set_middleware(const std::string& middle_ware) {
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) == NULL)
        std::cout << "Getting cwd failed..." << std::endl;

    std::string ws_root = get_ws_folder_from_benchmark_folder(std::string(cwd));
    set_env_dds_middleware(ws_root, middle_ware);
}

void log_first_to_last_node_latency_stats(
    const std::map<std::string, std::vector<double>>&
        first_to_last_node_latency_stats,
    std::ofstream& file) {
    file << first_to_last_node_latency_stats.at("end-to-end").at(0) << ",";
    file << first_to_last_node_latency_stats.at("end-to-end").at(1) << ",";
    file << first_to_last_node_latency_stats.at("rclcpp_interprocess_publish")
                .at(0)
         << ",";
    file << first_to_last_node_latency_stats.at("rclcpp_interprocess_publish")
                .at(1)
         << ",";
    file << first_to_last_node_latency_stats.at("rcl_publish").at(0) << ",";
    file << first_to_last_node_latency_stats.at("rcl_publish").at(1) << ",";
    file << first_to_last_node_latency_stats.at("rmw_publish").at(0) << ",";
    file << first_to_last_node_latency_stats.at("rmw_publish").at(1) << ",";
    file << first_to_last_node_latency_stats.at("dds_write").at(0) << ",";
    file << first_to_last_node_latency_stats.at("dds_write").at(1) << ",";
    file << first_to_last_node_latency_stats.at("sub_dds_on_data").at(0) << ",";
    file << first_to_last_node_latency_stats.at("sub_dds_on_data").at(1) << ",";
    file << first_to_last_node_latency_stats.at("sub_rclcpp_take_enter").at(0)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rclcpp_take_enter").at(1)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rcl_take_enter").at(0)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rcl_take_enter").at(1)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rmw_take_enter").at(0)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rmw_take_enter").at(1)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_dds_take_enter").at(0)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_dds_take_enter").at(1)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_dds_take_leave").at(0)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_dds_take_leave").at(1)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rcl_take_leave").at(0)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rcl_take_leave").at(1)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rmw_take_leave").at(0)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rmw_take_leave").at(1)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rclcpp_take_leave").at(0)
         << ",";
    file << first_to_last_node_latency_stats.at("sub_rclcpp_take_leave").at(1)
         << ",";
    file << first_to_last_node_latency_stats.at("rclcpp_handle").at(0) << ",";
    file << first_to_last_node_latency_stats.at("rclcpp_handle").at(1)
         << std::endl;
}

void add_latencies_header(std::ofstream& file) {
    file << "e2e_mean[us],"
         << "e2e_std[us],"
         << "rclcpp_interprocess_publish_mean[us],"
         << "rclcpp_interprocess_publish_std[us],"
         << "rcl_publish_mean[us],"
         << "rcl_publish_std[us],"
         << "rmw_publish_mean[us],"
         << "rmw_publish_std[us],"
         << "dds_write_mean[us],"
         << "dds_write_std[us],"
         << "sub_dds_on_data_mean[us],"
         << "sub_dds_on_data_std[us],"
         << "sub_rclcpp_take_enter_mean[us],"
         << "sub_rclcpp_take_enter_std[us],"
         << "sub_rcl_take_enter_mean[us],"
         << "sub_rcl_take_enter_std[us],"
         << "sub_rmw_take_enter_mean[us],"
         << "sub_rmw_take_enter_std[us],"
         << "sub_dds_take_enter_mean[us],"
         << "sub_dds_take_enter_std[us],"
         << "sub_dds_take_leave_mean[us],"
         << "sub_dds_take_leave_std[us],"
         << "sub_rmw_take_leave_mean[us],"
         << "sub_rmw_take_leave_std[us],"
         << "sub_rcl_take_leave_mean[us],"
         << "sub_rcl_take_leave_std[us],"
         << "sub_rclcpp_take_leave_mean[us],"
         << "sub_rclcpp_take_leave_std[us],"
         << "rclcpp_handle_mean[us],"
         << "rclcpp_handle_std[us]" << std::endl;
}
void save_nodes_vs_first_to_last_node_latencies_to_pgf_csv(
    const int no_nodes,
    const std::map<std::string, std::vector<double>>&
        first_to_last_node_latency_stats,
    std::ofstream& file) {
    file << no_nodes << ",";
    log_first_to_last_node_latency_stats(first_to_last_node_latency_stats,
                                         file);
}

void create_nodes_vs_first_to_last_node_latencies_haeder(std::ofstream& file) {
    file << "nodes,";
    add_latencies_header(file);
}

nlohmann::json create_json_for_system_info(
    const nlohmann::json& topology_j,
    const benchmark::Options& options) {
  nlohmann::json system_info_j;

  // create tracking options
  system_info_j["tracking_options"] = nlohmann::json();
  system_info_j["tracking_options"]["enabled"] =
      options.tracking_options.is_enabled;
  system_info_j["tracking_options"]["late_percentage"] =
      options.tracking_options.late_percentage;
  system_info_j["tracking_options"]["late_absolute_us"] =
      options.tracking_options.late_absolute_us;
  system_info_j["tracking_options"]["too_late_percentage"] =
      options.tracking_options.too_late_percentage;
  system_info_j["tracking_options"]["too_late_absolute_us"] =
      options.tracking_options.too_late_absolute_us;
  system_info_j["tracking_options"]["enable_profiling"] =
      options.tracking_options.enable_profiling;

  // create topology section
  system_info_j["topology"] = nlohmann::json();
  system_info_j["topology"]["data_processing_pipeline_range_start"] = options.data_processing_pipeline_range_start;
  system_info_j["topology"]["data_processing_pipeline_range_end"]   = options.data_processing_pipeline_range_end;
  system_info_j["topology"]["data_processing_pipeline_range_step_size"]   = options.data_processing_pipeline_range_step_size;
  system_info_j["topology"]["payload_file"]      = options.payload_file;
  if (options.msg_size != "")
    system_info_j["topology"]["msg_size"]      = options.msg_size;
  else
    system_info_j["topology"]["msg_size"] =
        topology_j["nodes"][0]["initial_pub"][0]["msg_type"];
  system_info_j["topology"]["filename"] = options.topology_json_list[0];
  system_info_j["topology"]["qos_reliability"] = options.qos_reliability;

  // create experiment section
  system_info_j["experiment"] = nlohmann::json();
  system_info_j["experiment"]["duration_sec"] = options.duration_sec;
  system_info_j["experiment"]["name"] = options.experiment_name;
  system_info_j["experiment"]["ipc_enabled"] = options.ipc;
  system_info_j["experiment"]["middle_ware"] = options.middle_ware;
  if (options.publish_frequency <= 0) {
    double period_ms = topology_j["nodes"][0]["initial_pub"][0]["period_ms"];
    system_info_j["experiment"]["publish_frequency"] = static_cast<int>(1e3/period_ms);
  } 
  else
    system_info_j["experiment"]["publish_frequency"] =
        options.publish_frequency;

  return system_info_j;
}

void save_system_info(const nlohmann::json& topology_j,
                      const std::string& directory, const std::string& filename,
                      const benchmark::Options& options) {
    nlohmann::json system_info_j =
        create_json_for_system_info(topology_j, options);
    std::ofstream json_file(directory + "/" + filename);
    json_file << std::setw(4) << system_info_j;
}
