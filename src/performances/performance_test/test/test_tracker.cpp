/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <gtest/gtest.h>
#include <algorithm>
#include <numeric>
#include <vector>

#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/stat.hpp"
#include "performance_test/ros2/tracker.hpp"
#include "performance_test_msgs/msg/performance_header.hpp"

#define RCL_NS_TO_US(ns) (ns / 1000.0)

class TestTracker : public ::testing::Test {
   public:
    const int PROFILE_IDX[14] = {PROFIDX_PUB_RCLCPP_INTERPROCESS_PUBLISH,
                                 PROFIDX_PUB_RCL_PUBLISH,
                                 PROFIDX_PUB_RMW_PUBLISH,
                                 PROFIDX_PUB_DDS_WRITE,

                                 PROFIDX_SUB_DDS_ONDATA,
                                 PROFIDX_SUB_RCLCPP_TAKE_ENTER,
                                 PROFIDX_SUB_RCL_TAKE_ENTER,
                                 PROFIDX_SUB_RMW_TAKE_ENTER,
                                 PROFIDX_SUB_DDS_TAKE_ENTER,
                                 PROFIDX_SUB_DDS_TAKE_LEAVE,
                                 PROFIDX_SUB_RMW_TAKE_LEAVE,
                                 PROFIDX_SUB_RCL_TAKE_LEAVE,
                                 PROFIDX_SUB_RCLCPP_TAKE_LEAVE,
                                 PROFIDX_SUB_RCLCPP_HANDLE};

    void assert_inter_node_latency_stats_equality(
        const std::map<std::string, performance_test::Stat<uint64_t>>& stats,
        const uint64_t* profiling_timestamps_absolute_ns,
        const uint64_t& msg_sent_ns);

    const double HEADER_SIZE = 10;
    const double FREQUENCY = 10;
    const char* topic_name_profiled = "chat_profile_ter";
    struct MsgDummy {
        performance_test_msgs::msg::PerformanceHeader header;
        uint64_t data[100];
    };
};
TEST_F(TestTracker, TrackerInitTest) {
    performance_test::Tracker tracker(
        "", "", performance_test::Tracker::TrackingOptions());

    ASSERT_EQ((unsigned long int)0, tracker.lost());
    ASSERT_EQ((unsigned long int)0, tracker.late());
    ASSERT_EQ((unsigned long int)0, tracker.too_late());
    ASSERT_EQ((unsigned long int)0, tracker.received());
    ASSERT_EQ((unsigned long int)0, tracker.last());

    EXPECT_TRUE(
        std::isnan(tracker.inter_node_latency_stats()["end-to-end"].mean()));
    EXPECT_TRUE(
        std::isnan(tracker.inter_node_latency_stats()["end-to-end"].stddev()));
    EXPECT_TRUE(
        std::isnan(tracker.inter_node_latency_stats()["end-to-end"].min()));
    EXPECT_TRUE(
        std::isnan(tracker.inter_node_latency_stats()["end-to-end"].max()));
    ASSERT_EQ((unsigned long int)0,
              tracker.inter_node_latency_stats()["end-to-end"].n());

    // only testing for mean for the ProfilingStats...
    EXPECT_TRUE(std::isnan(
        tracker.inter_node_latency_stats()["rclcpp_interprocess_publish"]
            .mean()));
    EXPECT_TRUE(
        std::isnan(tracker.inter_node_latency_stats()["rcl_publish"].mean()));
    EXPECT_TRUE(
        std::isnan(tracker.inter_node_latency_stats()["rmw_publish"].mean()));
    EXPECT_TRUE(
        std::isnan(tracker.inter_node_latency_stats()["dds_write"].mean()));

    EXPECT_TRUE(std::isnan(
        tracker.inter_node_latency_stats()["sub_dds_on_data"].mean()));
    EXPECT_TRUE(std::isnan(
        tracker.inter_node_latency_stats()["sub_rclcpp_take"].mean()));
    EXPECT_TRUE(
        std::isnan(tracker.inter_node_latency_stats()["sub_rcl_take"].mean()));
    EXPECT_TRUE(
        std::isnan(tracker.inter_node_latency_stats()["sub_dds_take"].mean()));

    EXPECT_TRUE(
        std::isnan(tracker.inter_node_latency_stats()["rclcpp_handle"].mean()));
    EXPECT_TRUE(std::isnan(
        tracker.inter_node_latency_stats()["sub_rmw_take_with_info"].mean()));
}

TEST_F(TestTracker, TrackerScanTest) {
    // tracker with disabled tracking options
    performance_test::Tracker tracker(
        "", "", performance_test::Tracker::TrackingOptions(false));

    auto header = performance_test_msgs::msg::PerformanceHeader();
    auto msg =
        std::make_shared<performance_test_msgs::msg::PerformanceHeader>(header);
    header.stamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

    rclcpp::Time t_now1(0, 10, RCL_SYSTEM_TIME);
    tracker.scan(header, t_now1, msg, nullptr);

    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(10),
                     tracker.inter_node_latency_stats()["end-to-end"].mean());
    ASSERT_DOUBLE_EQ((uint64_t)0,
                     tracker.inter_node_latency_stats()["end-to-end"].stddev());
    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(10),
                     tracker.inter_node_latency_stats()["end-to-end"].min());
    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(10),
                     tracker.inter_node_latency_stats()["end-to-end"].max());
    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(10), tracker.last());

    rclcpp::Time t_now2(0, 200, RCL_SYSTEM_TIME);
    tracker.scan(header, t_now2, msg, nullptr);

    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(105),
                     tracker.inter_node_latency_stats()["end-to-end"].mean());
    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(95),
                     tracker.inter_node_latency_stats()["end-to-end"].stddev());
    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(10),
                     tracker.inter_node_latency_stats()["end-to-end"].min());
    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(200),
                     tracker.inter_node_latency_stats()["end-to-end"].max());
    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(200), tracker.last());

    // This is 1e9 nanoseconds
    rclcpp::Time t_now3(1, 0, RCL_SYSTEM_TIME);
    tracker.scan(header, t_now3, msg, nullptr);

    EXPECT_NEAR((double)RCL_NS_TO_US(333333333.33333331),
                tracker.inter_node_latency_stats()["end-to-end"].mean(), 1e-1);
    EXPECT_NEAR((double)RCL_NS_TO_US(471404471.29356),
                tracker.inter_node_latency_stats()["end-to-end"].stddev(),
                1e-1);
    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(10),
                     tracker.inter_node_latency_stats()["end-to-end"].min());
    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(1e9),
                     tracker.inter_node_latency_stats()["end-to-end"].max());
    ASSERT_DOUBLE_EQ((uint64_t)RCL_NS_TO_US(1e9), tracker.last());
}

TEST_F(TestTracker, TrackingOptionsTest) {
    performance_test::Tracker::TrackingOptions t_options;
    t_options.late_percentage = 20;
    t_options.late_absolute_us = 5000;
    t_options.too_late_percentage = 100;
    t_options.too_late_absolute_us = 50000;

    performance_test::Tracker tracker("", "", t_options);

    auto header = performance_test_msgs::msg::PerformanceHeader();
    auto msg =
        std::make_shared<performance_test_msgs::msg::PerformanceHeader>(header);
    header.stamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    header.frequency = 100;

    // this message is on time
    rclcpp::Time t_now1(0, 1, RCL_SYSTEM_TIME);
    tracker.scan(header, t_now1, msg, nullptr);

    ASSERT_EQ((unsigned long int)0, tracker.late());
    ASSERT_EQ((unsigned long int)0, tracker.too_late());

    // this message is late because it exceeds late_percentage
    rclcpp::Time t_now2(0, 3e6, RCL_SYSTEM_TIME);
    tracker.scan(header, t_now2, msg, nullptr);

    ASSERT_EQ((unsigned long int)1, tracker.late());
    ASSERT_EQ((unsigned long int)0, tracker.too_late());

    // this message is late because it exceeds late_absolute_us
    rclcpp::Time t_now3(0, 6e6, RCL_SYSTEM_TIME);
    tracker.scan(header, t_now3, msg, nullptr);

    ASSERT_EQ((unsigned long int)2, tracker.late());
    ASSERT_EQ((unsigned long int)0, tracker.too_late());

    // this message is too late because it exceeds too_late_percentage
    rclcpp::Time t_now4(0, 11e6, RCL_SYSTEM_TIME);
    tracker.scan(header, t_now4, msg, nullptr);

    ASSERT_EQ((unsigned long int)2, tracker.late());
    ASSERT_EQ((unsigned long int)1, tracker.too_late());

    // this message is too late because it exceeds too_late_absolute_us
    rclcpp::Time t_now5(0, 111e6, RCL_SYSTEM_TIME);
    tracker.scan(header, t_now5, msg, nullptr);

    ASSERT_EQ((unsigned long int)2, tracker.late());
    ASSERT_EQ((unsigned long int)2, tracker.too_late());
}

TEST_F(TestTracker, Testinter_node_latency_statsUpdate_FastRTPS) {
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);

    performance_test::Tracker::TrackingOptions t_options;
    t_options.enable_profiling = true;

    // define constants here
    const int MSG_SENT_S = 0;
    const int MSG_SENT_NS = 5;

    uint64_t PROFILE_TIMESTAMPS_NS[14] = {1100,  1900,  3010,   4100,  4800,
                                          6000,  7000,  7500,   9000,  10500,
                                          11000, 12000, 130000, 140000};

    // create test environment
    rclcpp::init(0, NULL);
    auto dummy_node = std::make_shared<performance_test::Node>("dummy_node");
    performance_test::Tracker tracker("", "", t_options);
    auto profiled_msg = std::make_unique<MsgDummy>();

    // create profile
    for (int i = 0; i < sizeof(PROFILE_IDX) / sizeof(PROFILE_IDX[0]); i++) {
        store_profile(topic_name_profiled, profiled_msg.get(), PROFILE_IDX[i],
                      "", &PROFILE_TIMESTAMPS_NS[i]);
    }

    // just needed for compatibility reasons - not tested
    rclcpp::Time t0 = rclcpp::Time(MSG_SENT_S, MSG_SENT_NS, RCL_SYSTEM_TIME);
    profiled_msg->header.size = HEADER_SIZE;
    profiled_msg->header.frequency = FREQUENCY;
    profiled_msg->header.stamp = t0;

    tracker.scan(profiled_msg->header, t0, profiled_msg, nullptr);
    rclcpp::shutdown();

    assert_inter_node_latency_stats_equality(
        tracker.inter_node_latency_stats(), PROFILE_TIMESTAMPS_NS, MSG_SENT_NS);

    // check that we did not override anything in the header
    ASSERT_DOUBLE_EQ(tracker.size(), HEADER_SIZE);
    ASSERT_DOUBLE_EQ(tracker.frequency(), FREQUENCY);
    ASSERT_DOUBLE_EQ(tracker.inter_node_latency_stats()["end-to-end"].mean(),
                     0.0);
}

TEST_F(TestTracker, TestProfiling_DdsOnDataNoSkipForNonFastRTPSBackend) {
    setenv("RMW_IMPLEMENTATION", "rmw_connextdds", 1);

    performance_test::Tracker::TrackingOptions t_options;
    t_options.enable_profiling = true;

    // define constants here
    const int MSG_SENT_S = 0;
    const int MSG_SENT_NS = 5;

    uint64_t PROFILE_TIMESTAMPS_NS[14] = {1100,  1900,  3010,   4100,  5000,
                                          6000,  7000,  7500,   9000,  10500,
                                          11000, 12000, 130000, 140000};

    // create test environment
    rclcpp::init(0, NULL);
    auto dummy_node = std::make_shared<performance_test::Node>("dummy_node");
    performance_test::Tracker tracker("", "", t_options);
    auto profiled_msg = std::make_unique<MsgDummy>();

    // create profile
    for (int i = 0; i < sizeof(PROFILE_IDX) / sizeof(PROFILE_IDX[0]); i++) {
        store_profile(topic_name_profiled, profiled_msg.get(), PROFILE_IDX[i],
                      "", &PROFILE_TIMESTAMPS_NS[i]);
    }

    // just needed for compatibility reasons - not tested
    rclcpp::Time t0 = rclcpp::Time(MSG_SENT_S, MSG_SENT_NS, RCL_SYSTEM_TIME);
    profiled_msg->header.size = HEADER_SIZE;
    profiled_msg->header.frequency = FREQUENCY;
    profiled_msg->header.stamp = t0;

    tracker.scan(profiled_msg->header, t0, profiled_msg, nullptr);
    rclcpp::shutdown();

    assert_inter_node_latency_stats_equality(
        tracker.inter_node_latency_stats(), PROFILE_TIMESTAMPS_NS, MSG_SENT_NS);

    // check that we did not override anything in the header
    ASSERT_DOUBLE_EQ(tracker.size(), HEADER_SIZE);
    ASSERT_DOUBLE_EQ(tracker.frequency(), FREQUENCY);
    ASSERT_DOUBLE_EQ(tracker.inter_node_latency_stats()["end-to-end"].mean(),
                     0.0);

    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
}

void TestTracker::assert_inter_node_latency_stats_equality(
    const std::map<std::string, performance_test::Stat<uint64_t>>& stats,
    const uint64_t* profile_timestamps_absolute_ns,
    const uint64_t& msg_sent_ns) {
    ASSERT_EQ((uint64_t)RCL_NS_TO_US(
                  (profile_timestamps_absolute_ns[0] - msg_sent_ns)),
              stats.at("rclcpp_interprocess_publish").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[1] -
                                      profile_timestamps_absolute_ns[0])),
              stats.at("rcl_publish").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[2] -
                                      profile_timestamps_absolute_ns[1])),
              stats.at("rmw_publish").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[3] -
                                      profile_timestamps_absolute_ns[2])),
              stats.at("dds_write").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[4] -
                                      profile_timestamps_absolute_ns[3])),
              stats.at("sub_dds_on_data").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[5] -
                                      profile_timestamps_absolute_ns[4])),
              stats.at("sub_rclcpp_take_enter").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[6] -
                                      profile_timestamps_absolute_ns[5])),
              stats.at("sub_rcl_take_enter").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[7] -
                                      profile_timestamps_absolute_ns[6])),
              stats.at("sub_rmw_take_enter").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[8] -
                                      profile_timestamps_absolute_ns[7])),
              stats.at("sub_dds_take_enter").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[9] -
                                      profile_timestamps_absolute_ns[8])),
              stats.at("sub_dds_take_leave").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[10] -
                                      profile_timestamps_absolute_ns[9])),
              stats.at("sub_rmw_take_leave").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[11] -
                                      profile_timestamps_absolute_ns[10])),
              stats.at("sub_rcl_take_leave").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[12] -
                                      profile_timestamps_absolute_ns[11])),
              stats.at("sub_rclcpp_take_leave").mean());
    ASSERT_EQ((uint64_t)RCL_NS_TO_US((profile_timestamps_absolute_ns[13] -
                                      profile_timestamps_absolute_ns[12])),
              stats.at("rclcpp_handle").mean());
}