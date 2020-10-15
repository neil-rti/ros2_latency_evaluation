#ifndef __UTILS__
#define __UTILS__

#include <map>
#include <ros2profiling/profiling.h>

inline std::map<std::string, int> get_profiling_indices() {
    std::map<std::string, int> profiling_indices = {
                                          {"rclcpp_interprocess_publish", PROFIDX_PUB_RCLCPP_INTERPROCESS_PUBLISH},
                                          {"rcl_publish", PROFIDX_PUB_RCL_PUBLISH},
                                          {"rmw_publish", PROFIDX_PUB_RMW_PUBLISH},
                                          {"dds_write", PROFIDX_PUB_DDS_WRITE},
                                          {"sub_dds_on_data", PROFIDX_SUB_DDS_ONDATA},
                                          {"sub_rclcpp_take_enter", PROFIDX_SUB_RCLCPP_TAKE_ENTER},
                                          {"sub_rcl_take_enter", PROFIDX_SUB_RCL_TAKE_ENTER},
                                          {"sub_rmw_take_enter", PROFIDX_SUB_RMW_TAKE_ENTER},
                                          {"sub_dds_take_enter", PROFIDX_SUB_DDS_TAKE_ENTER},
                                          {"sub_dds_take_leave", PROFIDX_SUB_DDS_TAKE_LEAVE},
                                          {"sub_rmw_take_leave", PROFIDX_SUB_RMW_TAKE_LEAVE},
                                          {"sub_rcl_take_leave", PROFIDX_SUB_RCL_TAKE_LEAVE},
                                          {"sub_rclcpp_take_leave", PROFIDX_SUB_RCLCPP_TAKE_LEAVE},
                                          {"rclcpp_handle", PROFIDX_SUB_RCLCPP_HANDLE}};
    return profiling_indices;
}

inline std::vector<std::string> get_profiling_labels() {
    std::vector<std::string> profiling_indices = {
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
                                        "rclcpp_handle"};
    return profiling_indices;
}

template <typename T>
inline std::map<std::string, T> create_latency_stats_map_from_profiling_indices() {

    std::map<std::string, T> typed_map = {{"end-to-end", T()},
                                          {"rclcpp_interprocess_publish", T()},
                                          {"rcl_publish", T()},
                                          {"rmw_publish", T()},
                                          {"dds_write", T()},
                                          {"sub_dds_on_data", T()},
                                          {"sub_rclcpp_take_enter", T()},
                                          {"sub_rcl_take_enter", T()},
                                          {"sub_rmw_take_enter", T()},
                                          {"sub_dds_take_enter", T()},
                                          {"sub_dds_take_leave", T()},
                                          {"sub_rmw_take_leave", T()},
                                          {"sub_rcl_take_leave", T()},
                                          {"sub_rclcpp_take_leave", T()},
                                          {"rclcpp_handle", T()}};

    return typed_map;
}
#endif __UTILS__