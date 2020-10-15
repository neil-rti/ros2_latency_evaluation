/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */
#include <gtest/gtest.h>

#include "performance_test/ros2/node.hpp"
#include "performance_test/ros2/system.hpp"
#include "performance_test/ros2/tracker.hpp"
#include "performance_test_msgs/msg/performance_header.hpp"
#include "performance_test_msgs/msg/sample.hpp"
#include "performance_test_msgs/srv/sample.hpp"

class TestSystem : public ::testing::Test {
   public:
    static void SetUpTestCase() { rclcpp::init(0, nullptr); }

    void prepare_header(const int seconds, const int nanoseconds,
                        performance_test_msgs::msg::PerformanceHeader& header,
                        const int tracking_number = 0, const int size = 10,
                        const int frequency = 1) {
        header.stamp = rclcpp::Time(seconds, nanoseconds, RCL_SYSTEM_TIME);
        header.size = size;
        header.tracking_number = tracking_number;
        header.frequency = frequency;
    }

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
};

TEST_F(TestSystem, SystemAddNodesTest) {
    auto node_1 = std::make_shared<performance_test::Node>("node_1");
    auto node_2 = std::make_shared<performance_test::Node>("node_2");
    auto node_3 = std::make_shared<performance_test::Node>("node_3");
    std::vector<std::shared_ptr<performance_test::Node>> nodes_vec = {node_2,
                                                                      node_3};

    performance_test::System separate_threads_system;

    separate_threads_system.add_node(node_1);
    separate_threads_system.add_node(nodes_vec);
}

TEST_F(TestSystem, SystemPubSubTest) {
    auto pub_topic =
        performance_test::Topic<performance_test_msgs::msg::Sample>("my_topic");
    auto sub_topic =
        performance_test::Topic<performance_test_msgs::msg::Sample>("my_topic");

    int duration_sec = 1;
    performance_test::System ros2_system;

    // Create 1 pulisher node and 1 subscriber node
    auto pub_node = std::make_shared<performance_test::Node>("pub_node");
    pub_node->add_periodic_publisher(pub_topic, 10ms, PASS_BY_UNIQUE_PTR,
                                     rmw_qos_profile_default);
    ros2_system.add_node(pub_node);

    auto sub_node = std::make_shared<performance_test::Node>("sub_node");
    sub_node->add_subscriber(sub_topic, sub_topic, PASS_BY_SHARED_PTR,
                             performance_test::Tracker::TrackingOptions(),
                             rmw_qos_profile_default);
    ros2_system.add_node(sub_node);

    ros2_system.spin(duration_sec);

    auto trackers_vec_ptr = sub_node->all_trackers();
    auto tracker = (*trackers_vec_ptr)[0];

    ASSERT_EQ("my_topic", tracker.first);
    ASSERT_GT(tracker.second.received(), (unsigned long int)1);
}

/*TEST_F(TestSystem, SystemClientServerTest)
{
    auto service =
performance_test::Topic<performance_test_msgs::srv::Sample>("my_service");

    int duration_sec = 2;
    performance_test::System ros2_system;

    // Create 1 client node and 1 server node
    auto client_node = std::make_shared<performance_test::Node>("client_node");
    client_node->add_periodic_client(service, 10ms, rmw_qos_profile_default);
    ros2_system.add_node(client_node);

    auto server_node = std::make_shared<performance_test::Node>("server_node");
    server_node->add_server(service, rmw_qos_profile_default);
    ros2_system.add_node(server_node);

    // discovery check does not work with client/server yet
    ros2_system.spin(duration_sec, false);

    auto trackers_vec_ptr = client_node->all_trackers();
    auto tracker = (*trackers_vec_ptr)[0];

    ASSERT_EQ("my_service", tracker.first);
    ASSERT_GT(tracker.second.received(), (unsigned long int)1);
}

*/
TEST_F(TestSystem, SystemDifferentQoSTest) {
    auto pub_topic =
        performance_test::Topic<performance_test_msgs::msg::Sample>("my_topic");
    auto sub_topic =
        performance_test::Topic<performance_test_msgs::msg::Sample>("my_topic");

    int duration_sec = 1;
    performance_test::System ros2_system;
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

    // Create 1 pulisher node and 1 subscriber node
    auto pub_node = std::make_shared<performance_test::Node>("pub_node");
    qos_profile.reliability =
        rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    pub_node->add_periodic_publisher(pub_topic, 10ms, PASS_BY_UNIQUE_PTR,
                                     qos_profile);
    ros2_system.add_node(pub_node);

    auto sub_node = std::make_shared<performance_test::Node>("sub_node");
    qos_profile.reliability =
        rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    sub_node->add_subscriber(sub_topic, sub_topic, PASS_BY_SHARED_PTR,
                             performance_test::Tracker::TrackingOptions(),
                             qos_profile);
    ros2_system.add_node(sub_node);

    ros2_system.spin(duration_sec);

    auto trackers_vec_ptr = sub_node->all_trackers();
    auto tracker = (*trackers_vec_ptr)[0];

    // they have incompatible qos so they shouldn't communicate
    ASSERT_EQ(tracker.second.received(), (unsigned long int)0);
}

TEST_F(
    TestSystem,
    Test_FirstToLastNodeStatsCalculation_OnlyFirstToLastNode_FakedLatencies) {
    using namespace performance_test;
    const int DELAY_FIRST_LAP = 10000;
    const int DELAY_SECOND_LAP = 20000;

    // create meassages and trackers
    auto tracker_node2 =
        Tracker("node2", "srv_name", Tracker::TrackingOptions());
    auto tracker_node3 =
        Tracker("node3", "srv_name", Tracker::TrackingOptions());

    auto header1_2 = performance_test_msgs::msg::PerformanceHeader();
    auto msg1_2 =
        std::make_shared<performance_test_msgs::msg::PerformanceHeader>(
            header1_2);
    auto header2_3 = performance_test_msgs::msg::PerformanceHeader();
    auto msg2_3 =
        std::make_shared<performance_test_msgs::msg::PerformanceHeader>(
            header2_3);

    prepare_header(0, 0, header1_2);
    prepare_header(0, DELAY_FIRST_LAP, header2_3);

    // send msgs - first lap
    rclcpp::Time now(0, DELAY_FIRST_LAP, RCL_SYSTEM_TIME);
    tracker_node2.scan(header1_2, now, msg1_2, nullptr);

    now = rclcpp::Time(0, 2 * DELAY_FIRST_LAP, RCL_SYSTEM_TIME);
    tracker_node3.scan(header2_3, now, msg2_3, nullptr);

    // prepare headers etc for seconds lap
    prepare_header(0, 0, header1_2, 1);
    prepare_header(0, DELAY_SECOND_LAP, header2_3);

    // send msgs - second lap
    now = rclcpp::Time(0, DELAY_SECOND_LAP, RCL_SYSTEM_TIME);
    tracker_node2.scan(header1_2, now, msg1_2, nullptr);

    now = rclcpp::Time(0, 2 * DELAY_SECOND_LAP, RCL_SYSTEM_TIME);
    tracker_node3.scan(header2_3, now, msg2_3, nullptr);

    // imitate "all_trackers()"-getter
    typedef std::vector<std::pair<std::string, Tracker>> Trackers;
    auto trackers = std::make_shared<Trackers>(Trackers());

    trackers->push_back({"topic_node1_2", tracker_node2});
    trackers->push_back({"topic_node2_3", tracker_node3});

    System sys;
    ASSERT_DOUBLE_EQ(30, sys.calc_first_to_last_node_latency_stats(
                             trackers)["end-to-end"][0]);
    ASSERT_NEAR(
        14.142,
        sys.calc_first_to_last_node_latency_stats(trackers)["end-to-end"][1],
        0.01);
}

TEST_F(
    TestSystem,
    Test_FirstToLastNodeStatsCalculation_FirstToLastNodeLatencyIncludingProfiling_FakedLatencies) {
    using namespace performance_test;

    struct MsgDummy {
        performance_test_msgs::msg::PerformanceHeader header;
        uint64_t data[100];
    };

    const char* topic_name = "chat_profile_ter";
    const char* topic_name_second = "chat_profile_ter_second";

    uint64_t PROFILE_TIMESTAMPS_NS_FIRST_LAP[14] = {
        10000, 20000, 30000, 40000, 50000, 60000, 70000, 80000, 90000, 100000, 110000, 120000, 130000, 140000};
    uint64_t PROFILE_TIMESTAMPS_NS_SECOND_LAP[14] = {
        9000, 21000, 32000, 39000, 55000, 63000, 71000, 83000, 94000, 110000, 120000, 130000, 140000, 150000};

    const uint64_t DELAY_FIRST_LAP =
        150000;  // 150us - overhead to profile timestamps is 50us
    const uint64_t DELAY_SECOND_LAP = 200000;  // 200us, overhead is 90us

    // create node test environment
    auto dummy_node1 = std::make_shared<performance_test::Node>("dummy_node1");
    auto dummy_node2 = std::make_shared<performance_test::Node>("dummy_node2");
    auto dummy_node3 = std::make_shared<performance_test::Node>("dummy_node3");
    auto profiled_msg1_2 = std::make_unique<MsgDummy>();
    auto profiled_msg2_3 = std::make_unique<MsgDummy>();

    // and trackers
    Tracker::TrackingOptions opt = Tracker::TrackingOptions();
    opt.enable_profiling = true;
    auto tracker_node2 = Tracker("dummy_node2", "srv_name", opt);
    auto tracker_node3 = Tracker("dummy_node3", "srv_name", opt);

    // setup msg timestamps for first lap
    for (int i = 0; i < sizeof(PROFILE_IDX) / sizeof(PROFILE_IDX[0]); i++) {
        store_profile(topic_name, profiled_msg1_2.get(), PROFILE_IDX[i], "",
                      &PROFILE_TIMESTAMPS_NS_FIRST_LAP[i]);

        uint64_t profile_timestamp_second_node =
            PROFILE_TIMESTAMPS_NS_FIRST_LAP[i] + DELAY_FIRST_LAP;
        store_profile(topic_name_second, profiled_msg2_3.get(), PROFILE_IDX[i],
                      "", &profile_timestamp_second_node);
    }
    prepare_header(0, 0, profiled_msg1_2->header);
    prepare_header(0, DELAY_FIRST_LAP, profiled_msg2_3->header);

    // send msgs - first lap
    rclcpp::Time now(0, DELAY_FIRST_LAP, RCL_SYSTEM_TIME);
    tracker_node2.scan(profiled_msg1_2->header, now, profiled_msg1_2, nullptr);

    now = rclcpp::Time(0, 2 * DELAY_FIRST_LAP, RCL_SYSTEM_TIME);
    tracker_node3.scan(profiled_msg2_3->header, now, profiled_msg2_3, nullptr);

    // prepare headers etc for second lap
    for (int i = 0; i < sizeof(PROFILE_IDX) / sizeof(PROFILE_IDX[0]); i++) {
        store_profile(topic_name, profiled_msg1_2.get(), PROFILE_IDX[i], "",
                      &PROFILE_TIMESTAMPS_NS_SECOND_LAP[i]);

        uint64_t profile_timestamp_second_node =
            PROFILE_TIMESTAMPS_NS_SECOND_LAP[i] + DELAY_SECOND_LAP;
        store_profile(topic_name_second, profiled_msg2_3.get(), PROFILE_IDX[i],
                      "", &profile_timestamp_second_node);
    }

    profiled_msg2_3->header.stamp =
        rclcpp::Time(0, DELAY_SECOND_LAP, RCL_SYSTEM_TIME);

    profiled_msg1_2->header.tracking_number++;
    profiled_msg2_3->header.tracking_number++;

    // send msgs - second lap
    now = rclcpp::Time(0, DELAY_SECOND_LAP, RCL_SYSTEM_TIME);
    tracker_node2.scan(profiled_msg1_2->header, now, profiled_msg1_2, nullptr);

    now = rclcpp::Time(0, 2 * DELAY_SECOND_LAP, RCL_SYSTEM_TIME);
    tracker_node3.scan(profiled_msg2_3->header, now, profiled_msg2_3, nullptr);

    // imitate "all_trackers()"-getter
    typedef std::vector<std::pair<std::string, Tracker>> Trackers;
    auto trackers = std::make_shared<Trackers>(Trackers());

    trackers->push_back({"topic_node1_2", tracker_node2});
    trackers->push_back({"topic_node2_3", tracker_node3});

    System sys;

    // define expected values and perform test
    std::map<std::string, std::vector<double>> expected_stats = {
        {"end-to-end", {350, 70.7107}},
        {"rclcpp_interprocess_publish", {19, 1.414}},
        {"rcl_publish", {22, 2.828}},
        {"rmw_publish", {21, 1.414}},
        {"dds_write", {17, 4.2426}},
        {"sub_dds_on_data", {26, 8.4853}},
        {"sub_rclcpp_take_enter", {18, 2.828}},
        {"sub_rcl_take_enter", {18, 2.828}},
        {"sub_rmw_take_enter", {22, 2.828}},
        {"sub_dds_take_enter", {21, 1.414}},
        {"sub_dds_take_leave", {26, 8.4853}},
        {"sub_rmw_take_leave", {20, 0}},
        {"sub_rcl_take_leave", {20, 0}},
        {"sub_rclcpp_take_leave", {20, 0}},
        {"rclcpp_handle", {20, 0}}};
    std::map<std::string, std::vector<double>> actual_stats =
        sys.calc_first_to_last_node_latency_stats(trackers);

    for (const auto& latency_type_stats : actual_stats) {
        ASSERT_NEAR(latency_type_stats.second[0],
                    expected_stats[latency_type_stats.first][0], 0.01);
        ASSERT_NEAR(latency_type_stats.second[1],
                    expected_stats[latency_type_stats.first][1], 0.01);
    }
}

TEST_F(TestSystem,
       Test_FirstToLastNodeStatsCalculation_FirstToLastNodeLatency_TooLate) {
    using namespace performance_test;
    Tracker::TrackingOptions t_options;
    t_options.too_late_absolute_us = 5000;
    t_options.is_enabled = true;
    auto tracker_node2 = Tracker("node2", "srv_name", t_options);

    // create meassages
    auto header1_2 = performance_test_msgs::msg::PerformanceHeader();
    auto msg1_2 =
        std::make_shared<performance_test_msgs::msg::PerformanceHeader>(
            header1_2);

    prepare_header(0, 0, header1_2);

    // send msgs - first lap
    rclcpp::Time now(10, 0, RCL_SYSTEM_TIME);
    tracker_node2.scan(header1_2, now, msg1_2, nullptr);

    // imitate the "all_trackers()" getter method
    typedef std::vector<std::pair<std::string, Tracker>> Trackers;
    auto trackers = std::make_shared<Trackers>(Trackers());

    trackers->push_back({"topic_node1_2", tracker_node2});

    System sys;
    auto e2e_latency_stats =
        sys.calc_first_to_last_node_latency_stats(trackers);
    ASSERT_FALSE(std::isnan(e2e_latency_stats["end-to-end"][0]));
}
TEST_F(
    TestSystem,
    Test_FirstToLastNodeStatsCalculation_ProfilingValuesAreNanIfProfilingOff) {
    using namespace performance_test;
    Tracker::TrackingOptions t_options;
    t_options.is_enabled = true;
    t_options.enable_profiling = false;
    auto tracker_node2 = Tracker("node2", "srv_name", t_options);

    // create meassages
    auto header1_2 = performance_test_msgs::msg::PerformanceHeader();
    auto msg1_2 =
        std::make_shared<performance_test_msgs::msg::PerformanceHeader>(
            header1_2);

    prepare_header(0, 0, header1_2);

    // send msg
    rclcpp::Time now(1, 0, RCL_SYSTEM_TIME);
    tracker_node2.scan(header1_2, now, msg1_2, nullptr);

    // imitate the "all_trackers()" getter method
    typedef std::vector<std::pair<std::string, Tracker>> Trackers;
    auto trackers = std::make_shared<Trackers>(Trackers());

    trackers->push_back({"topic_node1_2", tracker_node2});

    System sys;
    auto e2e_latency_stats =
        sys.calc_first_to_last_node_latency_stats(trackers);

    for (const auto& latency_type : e2e_latency_stats) {
        if (latency_type.first != "end-to-end") {
            ASSERT_EQ((uint64_t)std::nanl(""), latency_type.second.at(0));
            ASSERT_EQ((uint64_t)std::nanl(""), latency_type.second.at(1));
        }
    }
}

TEST_F(
    TestSystem,
    Test_FirstToLastNodeStatsCalculation_ProfilingValuesAreNotNanMsgsTooLate) {
    using namespace performance_test;
    Tracker::TrackingOptions t_options;
    t_options.is_enabled = true;
    t_options.enable_profiling = true;
    t_options.too_late_absolute_us = 500;
    auto tracker_node2 = Tracker("node2", "srv_name", t_options);

    // create meassages
    auto header1_2 = performance_test_msgs::msg::PerformanceHeader();
    auto msg1_2 =
        std::make_shared<performance_test_msgs::msg::PerformanceHeader>(
            header1_2);
    prepare_header(0, 0, header1_2);

    // send msg
    rclcpp::Time now(1, 0, RCL_SYSTEM_TIME);
    tracker_node2.scan(header1_2, now, msg1_2, nullptr);

    // imitate the "all_trackers()" getter method
    typedef std::vector<std::pair<std::string, Tracker>> Trackers;
    auto trackers = std::make_shared<Trackers>(Trackers());

    trackers->push_back({"topic_node1_2", tracker_node2});

    System sys;
    auto e2e_latency_stats =
        sys.calc_first_to_last_node_latency_stats(trackers);

    // take those messages into consideration as well, therfore mean should be
    // nonzero. As we only have 1 sample however, std will be zero.
    for (const auto& latency_type : e2e_latency_stats) {
        ASSERT_NE((uint64_t)std::nanl(""), latency_type.second.at(0));
        ASSERT_EQ((uint64_t)std::nanl(""), latency_type.second.at(1));
    }
}

TEST_F(
    TestSystem,
    Test_FirstToLastNodeStatsLatCalculation_FourMessages_SecondAndThirdMessageTooLate_FirstToLastNodeLatency) {
    using namespace performance_test;
    const int DELAY_FIRST_LAP = 10;
    Tracker::TrackingOptions t_options;
    t_options.too_late_absolute_us = 5000;
    auto tracker_node2 = Tracker("node2", "srv_name", t_options);

    // create meassages
    auto header1_2 = performance_test_msgs::msg::PerformanceHeader();
    auto msg1_2 =
        std::make_shared<performance_test_msgs::msg::PerformanceHeader>(
            header1_2);

    prepare_header(0, 0, header1_2);

    // send msgs - first lap
    rclcpp::Time now(0, 10000, RCL_SYSTEM_TIME);
    tracker_node2.scan(header1_2, now, msg1_2, nullptr);

    // prepare messages for second lap
    prepare_header(0, 0, header1_2);

    // send msgs - second lap
    now = rclcpp::Time(20, 0, RCL_SYSTEM_TIME);
    tracker_node2.scan(header1_2, now, msg1_2, nullptr);

    // prepare messages for third lap
    prepare_header(0, 0, header1_2);

    // send msgs - third lap
    now = rclcpp::Time(20, 0, RCL_SYSTEM_TIME);
    tracker_node2.scan(header1_2, now, msg1_2, nullptr);

    // prepare messages for fourth lap
    prepare_header(0, 0, header1_2);

    // send msgs - fourth lap
    now = rclcpp::Time(0, 10000, RCL_SYSTEM_TIME);
    tracker_node2.scan(header1_2, now, msg1_2, nullptr);

    // imitate the "all_trackers()" getter method
    typedef std::vector<std::pair<std::string, Tracker>> Trackers;
    auto trackers = std::make_shared<Trackers>(Trackers());

    trackers->push_back({"topic_node1_2", tracker_node2});

    System sys;
    ASSERT_DOUBLE_EQ(10000005, sys.calc_first_to_last_node_latency_stats(
                                   trackers)["end-to-end"][0]);
    ASSERT_NEAR(
        11546999.61,
        sys.calc_first_to_last_node_latency_stats(trackers)["end-to-end"][1],
        0.01);
}

TEST_F(TestSystem, GetAllTrackersOfDataProcessingPipeline_CorrectNumberOfTrackers) {
    // create nodes and messages
    auto dummy_node1 = std::make_shared<performance_test::Node>("dummy_node1");
    auto dummy_node2 = std::make_shared<performance_test::Node>("dummy_node2");
    auto dummy_node3 = std::make_shared<performance_test::Node>("dummy_node3");

    auto profiled_msg1_2 =
        std::make_unique<performance_test_msgs::msg::Sample>();
    auto profiled_msg2_3 =
        std::make_unique<performance_test_msgs::msg::Sample>();

    auto topic1_2 =
        performance_test::Topic<performance_test_msgs::msg::Sample>("topic1_2");
    auto topic2_3 =
        performance_test::Topic<performance_test_msgs::msg::Sample>("topic2_3");

    // add publishers and subscribers
    dummy_node1->add_periodic_publisher(topic1_2, std::chrono::milliseconds(10),
                                        PASS_BY_SHARED_PTR);
    dummy_node2->add_non_periodic_publisher(topic1_2, topic2_3,
                                            PASS_BY_SHARED_PTR);

    dummy_node2->add_subscriber(topic1_2, topic2_3, PASS_BY_SHARED_PTR);
    dummy_node3->add_subscriber(topic2_3, topic2_3, PASS_BY_SHARED_PTR);

    // now create the system
    performance_test::System sys;
    sys.add_node({dummy_node1, dummy_node2, dummy_node3});

    // define expected values and perform test
    ASSERT_EQ(2, sys.get_trackers_of_node_system()->size());
}